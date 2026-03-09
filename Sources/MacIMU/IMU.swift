import CoreFoundation
import Foundation
import IOKit
import IOKit.hid

private let pageVendor: Int = 0xFF00
private let pageSensor: Int = 0x0020
private let usageAccel: Int = 3
private let usageGyro: Int = 9
private let usageALS: Int = 4
private let usageLid: Int = 138

private let imuReportLength = 22
private let imuDataOffset = 6
private let alsReportLength = 122
private let lidReportLength = 3
private let reportBufferSize = 4096

private let accelScale = 65536.0
private let gyroScale = 65536.0
private let nativeRateHz = 800
private let defaultDecimation = 8

private enum SensorKind: String {
    case accel
    case gyro
    case als
    case lid
}

private final class CallbackContext {
    weak var imu: IMU?
    let sensorKind: SensorKind

    init(imu: IMU, sensorKind: SensorKind) {
        self.imu = imu
        self.sensorKind = sensorKind
    }
}

private final class DeviceHandle {
    let service: io_service_t
    let device: IOHIDDevice
    let sensorKind: SensorKind
    let context: CallbackContext
    let reportBuffer: UnsafeMutablePointer<UInt8>

    init(service: io_service_t, device: IOHIDDevice, sensorKind: SensorKind, context: CallbackContext, reportBuffer: UnsafeMutablePointer<UInt8>) {
        self.service = service
        self.device = device
        self.sensorKind = sensorKind
        self.context = context
        self.reportBuffer = reportBuffer
    }

    deinit {
        reportBuffer.deallocate()
    }
}

public final class IMU: @unchecked Sendable {
    private let wantAccel: Bool
    private let wantGyro: Bool
    private let wantALS: Bool
    private let wantLid: Bool
    private let wantOrientation: Bool
    private let decimation: Int
    private let maxBufferedSamples = 8000

    private let lock = NSLock()
    private var isStarted = false
    private var shutdownRequested = false
    private var workerThread: Thread?
    private var runLoop: CFRunLoop?
    private var deviceHandles: [DeviceHandle] = []

    private var accelSamples: [TimedSample] = []
    private var gyroSamples: [TimedSample] = []
    private var accelReadIndex = 0
    private var gyroReadIndex = 0
    private var latestALSReading: ALSReading?
    private var latestLidAngle: Double?
    private var lastAccelSample: TimedSample?
    private var lastGyroSample: TimedSample?
    private var latestOrientation: Orientation?
    private var sampleCount = 0
    private var startUptime = 0.0

    private var accelDecimationCount = 0
    private var gyroDecimationCount = 0
    private var ahrs: MahonyAHRS?

    public init(
        accel: Bool = true,
        gyro: Bool = true,
        als: Bool = false,
        lid: Bool = false,
        orientation: Bool = false,
        decimation: Int? = nil,
        sampleRate: Int? = nil
    ) {
        if decimation != nil && sampleRate != nil {
            fatalError("Specify decimation or sampleRate, not both.")
        }

        let effectiveDecimation: Int
        if let sampleRate {
            effectiveDecimation = max(1, Int(round(Double(nativeRateHz) / Double(sampleRate))))
        } else {
            effectiveDecimation = decimation ?? defaultDecimation
        }

        if orientation && (!accel || !gyro) {
            fatalError("orientation=true requires accel=true and gyro=true")
        }

        self.wantAccel = accel
        self.wantGyro = gyro
        self.wantALS = als
        self.wantLid = lid
        self.wantOrientation = orientation
        self.decimation = max(1, effectiveDecimation)
        if orientation {
            self.ahrs = MahonyAHRS()
        }
    }

    deinit {
        stop()
    }

    public static func available() -> Bool {
        enumerateSPUServices().contains { $0.usagePage == pageVendor && $0.usage == usageAccel }
    }

    public static func deviceInfo() -> DeviceInfo {
        var info = DeviceInfo()

        for entry in enumerateSPUServices() {
            switch (entry.usagePage, entry.usage) {
            case (pageVendor, usageAccel):
                info.sensors.append("accelerometer")
            case (pageVendor, usageGyro):
                info.sensors.append("gyroscope")
            case (pageVendor, usageALS):
                info.sensors.append("ambient_light")
            case (pageSensor, usageLid):
                info.sensors.append("lid_angle")
            default:
                break
            }

            if info.product == nil { info.product = entry.product }
            if info.serialNumber == nil { info.serialNumber = entry.serialNumber }
            if info.manufacturer == nil { info.manufacturer = entry.manufacturer }
            if info.transport == nil { info.transport = entry.transport }
            if info.vendorID == nil { info.vendorID = entry.vendorID }
            if info.productID == nil { info.productID = entry.productID }
        }
        return info
    }

    public var isRunning: Bool {
        lock.withLock {
            isStarted
        }
    }

    public var effectiveSampleRate: Double? {
        lock.withLock {
            guard isStarted else {
                return nil
            }
            let elapsed = ProcessInfo.processInfo.systemUptime - startUptime
            guard elapsed >= 0.5, sampleCount >= 10 else {
                return nil
            }
            return Double(sampleCount) / elapsed
        }
    }

    public func start() throws {
        if getuid() != 0 {
            throw IMUError.permissionDenied
        }
        if !Self.available() {
            throw IMUError.sensorNotFound
        }

        lock.withLock {
            guard !isStarted else {
                return
            }
            shutdownRequested = false
            startUptime = ProcessInfo.processInfo.systemUptime
            isStarted = true
            sampleCount = 0
            accelDecimationCount = 0
            gyroDecimationCount = 0
            if wantOrientation {
                ahrs?.reset()
                latestOrientation = nil
            }
        }

        let thread = Thread { [weak self] in
            self?.runSensorLoop()
        }
        thread.name = "MacIMU.SensorLoop"
        workerThread = thread
        thread.start()
    }

    public func stop() {
        var thread: Thread?
        var runLoopToStop: CFRunLoop?

        lock.withLock {
            guard isStarted else {
                return
            }
            shutdownRequested = true
            thread = workerThread
            runLoopToStop = runLoop
        }

        if let runLoopToStop {
            CFRunLoopStop(runLoopToStop)
            CFRunLoopWakeUp(runLoopToStop)
        }

        while let thread, !thread.isFinished {
            Thread.sleep(forTimeInterval: 0.01)
        }

        lock.withLock {
            workerThread = nil
            runLoop = nil
            deviceHandles.removeAll()
            isStarted = false
        }
    }

    public func readAccel() -> [Sample] {
        lock.withLock {
            let result = Array(accelSamples[accelReadIndex...]).map { Sample(x: $0.x, y: $0.y, z: $0.z) }
            accelReadIndex = accelSamples.count
            return result
        }
    }

    public func readGyro() -> [Sample] {
        lock.withLock {
            let result = Array(gyroSamples[gyroReadIndex...]).map { Sample(x: $0.x, y: $0.y, z: $0.z) }
            gyroReadIndex = gyroSamples.count
            return result
        }
    }

    public func readAccelTimed() -> [TimedSample] {
        lock.withLock {
            let result = Array(accelSamples[accelReadIndex...])
            accelReadIndex = accelSamples.count
            return result
        }
    }

    public func readGyroTimed() -> [TimedSample] {
        lock.withLock {
            let result = Array(gyroSamples[gyroReadIndex...])
            gyroReadIndex = gyroSamples.count
            return result
        }
    }

    public func latestAccel() -> Sample? {
        lock.withLock {
            guard let sample = accelSamples.last else {
                return nil
            }
            return Sample(x: sample.x, y: sample.y, z: sample.z)
        }
    }

    public func latestGyro() -> Sample? {
        lock.withLock {
            guard let sample = gyroSamples.last else {
                return nil
            }
            return Sample(x: sample.x, y: sample.y, z: sample.z)
        }
    }

    public func orientation() -> Orientation? {
        lock.withLock {
            latestOrientation
        }
    }

    public func readALS() -> ALSReading? {
        lock.withLock {
            latestALSReading
        }
    }

    public func readLid() -> Double? {
        lock.withLock {
            latestLidAngle
        }
    }

    public func readAll() -> [String: Any] {
        lock.withLock {
            var result: [String: Any] = [:]
            if let accel = accelSamples.last {
                result["accel"] = Sample(x: accel.x, y: accel.y, z: accel.z)
            }
            if let gyro = gyroSamples.last {
                result["gyro"] = Sample(x: gyro.x, y: gyro.y, z: gyro.z)
            }
            if let lid = latestLidAngle {
                result["lid"] = lid
            }
            if let als = latestALSReading {
                result["als"] = als
            }
            if let latestOrientation {
                result["orientation"] = latestOrientation
            }
            return result
        }
    }

    private func runSensorLoop() {
        guard let currentRunLoop = CFRunLoopGetCurrent() else {
            return
        }
        lock.withLock {
            runLoop = currentRunLoop
        }

        wakeSPUDrivers()

        var openedHandles: [DeviceHandle] = []
        let runLoopMode = CFRunLoopMode.defaultMode.rawValue as CFString
        for entry in enumerateSPUServices() {
            guard let kind = sensorKind(for: entry.usagePage, usage: entry.usage) else {
                continue
            }
            if !isEnabled(kind: kind) {
                continue
            }

            guard let device = IOHIDDeviceCreate(kCFAllocatorDefault, entry.service) else {
                continue
            }
            let result = IOHIDDeviceOpen(device, IOOptionBits(kIOHIDOptionsTypeNone))
            if result != kIOReturnSuccess {
                continue
            }

            let buffer = UnsafeMutablePointer<UInt8>.allocate(capacity: reportBufferSize)
            buffer.initialize(repeating: 0, count: reportBufferSize)
            let context = CallbackContext(imu: self, sensorKind: kind)
            let opaque = Unmanaged.passUnretained(context).toOpaque()

            IOHIDDeviceRegisterInputReportCallback(
                device,
                buffer,
                reportBufferSize,
                hidReportCallback,
                opaque
            )
            IOHIDDeviceScheduleWithRunLoop(device, currentRunLoop, runLoopMode)
            openedHandles.append(DeviceHandle(service: entry.service, device: device, sensorKind: kind, context: context, reportBuffer: buffer))
        }

        lock.withLock {
            deviceHandles = openedHandles
        }

        while true {
            let shouldStop = lock.withLock { shutdownRequested }
            if shouldStop {
                break
            }
            CFRunLoopRunInMode(.defaultMode, 0.25, false)
        }

        for handle in openedHandles {
            IOHIDDeviceUnscheduleFromRunLoop(handle.device, currentRunLoop, runLoopMode)
            IOHIDDeviceClose(handle.device, IOOptionBits(kIOHIDOptionsTypeNone))
            IOObjectRelease(handle.service)
        }
    }

    private func isEnabled(kind: SensorKind) -> Bool {
        switch kind {
        case .accel:
            return wantAccel
        case .gyro:
            return wantGyro
        case .als:
            return wantALS
        case .lid:
            return wantLid
        }
    }

    fileprivate func processReport(kind: SensorKind, report: UnsafePointer<UInt8>, length: Int) {
        let timestamp = ProcessInfo.processInfo.systemUptime

        switch kind {
        case .accel:
            guard length == imuReportLength else { return }
            lock.withLock {
                accelDecimationCount += 1
                if accelDecimationCount < decimation {
                    return
                }
                accelDecimationCount = 0
            }

            let sample = TimedSample(
                t: timestamp,
                x: Double(int32LE(report, offset: imuDataOffset)) / accelScale,
                y: Double(int32LE(report, offset: imuDataOffset + 4)) / accelScale,
                z: Double(int32LE(report, offset: imuDataOffset + 8)) / accelScale
            )
            handleAccel(sample: sample)

        case .gyro:
            guard length == imuReportLength else { return }
            lock.withLock {
                gyroDecimationCount += 1
                if gyroDecimationCount < decimation {
                    return
                }
                gyroDecimationCount = 0
            }

            let sample = TimedSample(
                t: timestamp,
                x: Double(int32LE(report, offset: imuDataOffset)) / gyroScale,
                y: Double(int32LE(report, offset: imuDataOffset + 4)) / gyroScale,
                z: Double(int32LE(report, offset: imuDataOffset + 8)) / gyroScale
            )
            handleGyro(sample: sample)

        case .als:
            guard length == alsReportLength else { return }
            let lux = Double(float32LE(report, offset: 40))
            let channels = [
                uint32LE(report, offset: 20),
                uint32LE(report, offset: 24),
                uint32LE(report, offset: 28),
                uint32LE(report, offset: 32),
            ]
            lock.withLock {
                latestALSReading = ALSReading(lux: lux, channels: channels)
            }

        case .lid:
            guard length >= lidReportLength, report[0] == 1 else { return }
            let raw = UInt16(report[1]) | (UInt16(report[2]) << 8)
            let angle = Double(raw & 0x1FF)
            lock.withLock {
                latestLidAngle = angle
            }
        }
    }

    private func handleAccel(sample: TimedSample) {
        lock.withLock {
            let previousAccelSample = lastAccelSample
            accelSamples.append(sample)
            if accelSamples.count > maxBufferedSamples {
                accelSamples.removeFirst()
                accelReadIndex = max(0, accelReadIndex - 1)
            }
            lastAccelSample = sample
            sampleCount += 1

            if wantOrientation, let lastGyroSample {
                let dt = max(0.001, sample.t - (previousAccelSample?.t ?? sample.t))
                ahrs?.update(
                    ax: sample.x,
                    ay: sample.y,
                    az: sample.z,
                    gxDps: lastGyroSample.x,
                    gyDps: lastGyroSample.y,
                    gzDps: lastGyroSample.z,
                    dt: dt
                )
                if let ahrs {
                    let euler = ahrs.euler()
                    let q = ahrs.quaternion
                    latestOrientation = Orientation(
                        roll: euler.0,
                        pitch: euler.1,
                        yaw: euler.2,
                        qw: q.0,
                        qx: q.1,
                        qy: q.2,
                        qz: q.3
                    )
                }
            }
        }
    }

    private func handleGyro(sample: TimedSample) {
        lock.withLock {
            gyroSamples.append(sample)
            if gyroSamples.count > maxBufferedSamples {
                gyroSamples.removeFirst()
                gyroReadIndex = max(0, gyroReadIndex - 1)
            }
            lastGyroSample = sample
        }
    }

    private func sensorKind(for usagePage: Int, usage: Int) -> SensorKind? {
        switch (usagePage, usage) {
        case (pageVendor, usageAccel):
            return .accel
        case (pageVendor, usageGyro):
            return .gyro
        case (pageVendor, usageALS):
            return .als
        case (pageSensor, usageLid):
            return .lid
        default:
            return nil
        }
    }

    private func wakeSPUDrivers() {
        guard let matching = IOServiceMatching("AppleSPUHIDDriver") else {
            return
        }

        var iterator: io_iterator_t = 0
        guard IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iterator) == KERN_SUCCESS else {
            return
        }
        defer { IOObjectRelease(iterator) }

        while true {
            let service = IOIteratorNext(iterator)
            if service == 0 {
                break
            }
            defer { IOObjectRelease(service) }

            IORegistryEntrySetCFProperty(service, "SensorPropertyReportingState" as CFString, 1 as CFTypeRef)
            IORegistryEntrySetCFProperty(service, "SensorPropertyPowerState" as CFString, 1 as CFTypeRef)
            IORegistryEntrySetCFProperty(service, "ReportInterval" as CFString, 1000 as CFTypeRef)
        }
    }

}

private let hidReportCallback: IOHIDReportCallback = { context, _, _, _, _, report, reportLength in
    guard let context else {
        return
    }

    let callbackContext = Unmanaged<CallbackContext>.fromOpaque(context).takeUnretainedValue()
    callbackContext.imu?.processReport(kind: callbackContext.sensorKind, report: UnsafePointer(report), length: reportLength)
}

private struct ServiceEntry {
    let service: io_service_t
    let usagePage: Int
    let usage: Int
    let product: String?
    let serialNumber: String?
    let manufacturer: String?
    let transport: String?
    let vendorID: Int?
    let productID: Int?
}

private func enumerateSPUServices() -> [ServiceEntry] {
    guard let matching = IOServiceMatching("AppleSPUHIDDevice") else {
        return []
    }

    var iterator: io_iterator_t = 0
    guard IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iterator) == KERN_SUCCESS else {
        return []
    }
    defer { IOObjectRelease(iterator) }

    var entries: [ServiceEntry] = []
    while true {
        let service = IOIteratorNext(iterator)
        if service == 0 {
            break
        }

        let usagePage = intProperty(service: service, key: "PrimaryUsagePage") ?? 0
        let usage = intProperty(service: service, key: "PrimaryUsage") ?? 0
        let entry = ServiceEntry(
            service: service,
            usagePage: usagePage,
            usage: usage,
            product: stringProperty(service: service, key: "Product"),
            serialNumber: stringProperty(service: service, key: "SerialNumber"),
            manufacturer: stringProperty(service: service, key: "Manufacturer"),
            transport: stringProperty(service: service, key: "Transport"),
            vendorID: intProperty(service: service, key: "VendorID"),
            productID: intProperty(service: service, key: "ProductID")
        )
        entries.append(entry)
    }
    return entries
}

private func stringProperty(service: io_registry_entry_t, key: String) -> String? {
    let value = IORegistryEntryCreateCFProperty(service, key as CFString, kCFAllocatorDefault, 0)?.takeRetainedValue()
    return value as? String
}

private func intProperty(service: io_registry_entry_t, key: String) -> Int? {
    let value = IORegistryEntryCreateCFProperty(service, key as CFString, kCFAllocatorDefault, 0)?.takeRetainedValue()
    switch value {
    case let number as NSNumber:
        return number.intValue
    default:
        return nil
    }
}

private func int32LE(_ bytes: UnsafePointer<UInt8>, offset: Int) -> Int32 {
    let raw = UInt32(bytes[offset])
        | (UInt32(bytes[offset + 1]) << 8)
        | (UInt32(bytes[offset + 2]) << 16)
        | (UInt32(bytes[offset + 3]) << 24)
    return Int32(bitPattern: raw)
}

private func uint32LE(_ bytes: UnsafePointer<UInt8>, offset: Int) -> UInt32 {
    UInt32(bytes[offset])
        | (UInt32(bytes[offset + 1]) << 8)
        | (UInt32(bytes[offset + 2]) << 16)
        | (UInt32(bytes[offset + 3]) << 24)
}

private func float32LE(_ bytes: UnsafePointer<UInt8>, offset: Int) -> Float {
    Float(bitPattern: uint32LE(bytes, offset: offset))
}

private extension NSLock {
    func withLock<T>(_ body: () -> T) -> T {
        lock()
        defer { unlock() }
        return body()
    }
}
