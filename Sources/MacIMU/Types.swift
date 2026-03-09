import Foundation

public struct Sample: Sendable {
    public let x: Double
    public let y: Double
    public let z: Double

    public init(x: Double, y: Double, z: Double) {
        self.x = x
        self.y = y
        self.z = z
    }
}

public struct TimedSample: Sendable {
    public let t: Double
    public let x: Double
    public let y: Double
    public let z: Double

    public init(t: Double, x: Double, y: Double, z: Double) {
        self.t = t
        self.x = x
        self.y = y
        self.z = z
    }
}

public struct ALSReading: Sendable {
    public let lux: Double
    public let channels: [UInt32]

    public init(lux: Double, channels: [UInt32]) {
        self.lux = lux
        self.channels = channels
    }
}

public struct Orientation: Sendable {
    public let roll: Double
    public let pitch: Double
    public let yaw: Double
    public let qw: Double
    public let qx: Double
    public let qy: Double
    public let qz: Double

    public init(roll: Double, pitch: Double, yaw: Double, qw: Double, qx: Double, qy: Double, qz: Double) {
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.qw = qw
        self.qx = qx
        self.qy = qy
        self.qz = qz
    }
}

public struct DeviceInfo: Sendable {
    public var sensors: [String]
    public var product: String?
    public var serialNumber: String?
    public var manufacturer: String?
    public var transport: String?
    public var vendorID: Int?
    public var productID: Int?

    public init(
        sensors: [String] = [],
        product: String? = nil,
        serialNumber: String? = nil,
        manufacturer: String? = nil,
        transport: String? = nil,
        vendorID: Int? = nil,
        productID: Int? = nil
    ) {
        self.sensors = sensors
        self.product = product
        self.serialNumber = serialNumber
        self.manufacturer = manufacturer
        self.transport = transport
        self.vendorID = vendorID
        self.productID = productID
    }
}

public enum IMUError: Error, LocalizedError {
    case permissionDenied
    case sensorNotFound
    case failedToOpenDevice(String)

    public var errorDescription: String? {
        switch self {
        case .permissionDenied:
            return "MacIMU requires root privileges. Run with sudo."
        case .sensorNotFound:
            return "No Apple SPU IMU device was found on this machine."
        case .failedToOpenDevice(let detail):
            return "Failed to open HID device: \(detail)"
        }
    }
}
