import Foundation
import Dispatch
import Darwin
import MacIMU

enum ANSI {
    static let reset = "\u{001B}[0m"
    static let bold = "\u{001B}[1m"
    static let dim = "\u{001B}[2m"
    static let red = "\u{001B}[31m"
    static let green = "\u{001B}[32m"
    static let yellow = "\u{001B}[33m"
    static let cyan = "\u{001B}[36m"
    static let brightRed = "\u{001B}[91m"
    static let brightWhite = "\u{001B}[97m"
    static let brightCyan = "\u{001B}[96m"
    static let hideCursor = "\u{001B}[?25l"
    static let showCursor = "\u{001B}[?25h"
    static let enterAlt = "\u{001B}[?1049h"
    static let exitAlt = "\u{001B}[?1049l"
    static let clear = "\u{001B}[2J\u{001B}[H"
}

private let terminalWidth = 76
private let sparkBlocks = Array(" ▁▂▃▄▅▆▇█")

struct MotionConfig {
    var useKBPulse = true
    var kbpulseBinary: String?
    var kbpulseAsRoot = false
    var duration: Double?
    var sampleRate = 100
}

struct MotionEvent: Codable {
    let timestamp: Double
    let displayTime: String
    let severity: String
    let symbol: String
    let label: String
    let amplitude: Double
    let sources: [String]
    let bands: [String]
}

struct EventLog: Codable {
    let generated: String
    let totalSamples: Int
    let events: [MotionEvent]
}

final class LimitedBuffer<Element> {
    private(set) var values: [Element] = []
    let capacity: Int

    init(capacity: Int) {
        self.capacity = capacity
    }

    func append(_ value: Element) {
        values.append(value)
        if values.count > capacity {
            values.removeFirst(values.count - capacity)
        }
    }

    func removeAll() {
        values.removeAll(keepingCapacity: true)
    }
}

final class StopFlag {
    private let lock = NSLock()
    private var stopped = false

    func stop() {
        lock.lock()
        stopped = true
        lock.unlock()
    }

    var isStopped: Bool {
        lock.lock()
        defer { lock.unlock() }
        return stopped
    }
}

final class VibrationDetector {
    let fs: Double
    var sampleCount = 0

    private let hpAlpha = 0.95
    private var hpPrevRaw = (0.0, 0.0, 0.0)
    private var hpPrevOut = (0.0, 0.0, 0.0)
    private var hpReady = false

    let waveform: LimitedBuffer<Double>
    let waveformXYZ: LimitedBuffer<Sample>
    let dwtBuffer: LimitedBuffer<Double>
    let rmsTrend: LimitedBuffer<Double>
    let hrBuffer: LimitedBuffer<Double>
    let kurtBuffer: LimitedBuffer<Double>
    let peakBuffer: LimitedBuffer<Double>
    let eventTimes: LimitedBuffer<Double>
    var bandEnergy: [LimitedBuffer<Double>]
    var staLtaRing: [LimitedBuffer<Double>]

    var latestRaw = Sample(x: 0, y: 0, z: 0)
    var latestMagnitude = 0.0
    var gyroLatest = Sample(x: 0, y: 0, z: 0)

    private var sta = [0.0, 0.0, 0.0]
    private var lta = [1e-10, 1e-10, 1e-10]
    private let staN = [3.0, 15.0, 50.0]
    private let ltaN = [100.0, 500.0, 2000.0]
    private let staThreshOn = [3.0, 2.5, 2.0]
    private let staThreshOff = [1.5, 1.3, 1.2]
    private var staActive = [false, false, false]
    var staLatest = [1.0, 1.0, 1.0]

    private var cusumPos = 0.0
    private var cusumNeg = 0.0
    private var cusumMu = 0.0
    private let cusumK = 0.0005
    private let cusumH = 0.01
    var cusumValue = 0.0

    var kurtosis = 3.0
    var crest = 1.0
    var rms = 0.0
    var peak = 0.0
    var madSigma = 0.0

    var period: Double?
    var periodStd: Double?
    var periodCV: Double?
    var periodFreq: Double?
    var acorrRing: [Double] = []

    var hrBPM: Double?
    var hrConfidence = 0.0

    private let hrHPAlpha: Double
    private let hrLPAlpha: Double
    private var hrHPPrevIn = 0.0
    private var hrHPPrevOut = 0.0
    private var hrLPPrev = 0.0

    private let ahrs = MahonyAHRS()
    private var lastOrientation: Orientation?

    private var rmsWindow: [Double] = []
    private var lastEventTime = 0.0

    let bandLabels = ["50Hz", "25Hz", "12Hz", " 6Hz", " 3Hz"]
    var events: [MotionEvent] = []

    init(sampleRate: Int) {
        self.fs = Double(sampleRate)
        self.waveform = LimitedBuffer(capacity: sampleRate * 5)
        self.waveformXYZ = LimitedBuffer(capacity: sampleRate * 5)
        self.dwtBuffer = LimitedBuffer(capacity: 512)
        self.rmsTrend = LimitedBuffer(capacity: 100)
        self.hrBuffer = LimitedBuffer(capacity: sampleRate * 10)
        self.kurtBuffer = LimitedBuffer(capacity: sampleRate)
        self.peakBuffer = LimitedBuffer(capacity: sampleRate * 2)
        self.eventTimes = LimitedBuffer(capacity: 200)
        self.bandEnergy = (0..<5).map { _ in LimitedBuffer<Double>(capacity: 50) }
        self.staLtaRing = (0..<3).map { _ in LimitedBuffer<Double>(capacity: 30) }
        self.hrHPAlpha = fs / (fs + 2.0 * .pi * 0.8)
        self.hrLPAlpha = 2.0 * .pi * 3.0 / (2.0 * .pi * 3.0 + fs)
    }

    func processGyro(_ sample: Sample) {
        gyroLatest = sample
    }

    @discardableResult
    func processAccel(_ sample: TimedSample) -> Double {
        sampleCount += 1
        latestRaw = Sample(x: sample.x, y: sample.y, z: sample.z)
        latestMagnitude = magnitude(x: sample.x, y: sample.y, z: sample.z)
        updateOrientation(sample)

        if !hpReady {
            hpPrevRaw = (sample.x, sample.y, sample.z)
            hpPrevOut = (0, 0, 0)
            hpReady = true
            waveform.append(0)
            waveformXYZ.append(Sample(x: 0, y: 0, z: 0))
            dwtBuffer.append(0)
            return 0
        }

        let hx = hpAlpha * (hpPrevOut.0 + sample.x - hpPrevRaw.0)
        let hy = hpAlpha * (hpPrevOut.1 + sample.y - hpPrevRaw.1)
        let hz = hpAlpha * (hpPrevOut.2 + sample.z - hpPrevRaw.2)
        hpPrevRaw = (sample.x, sample.y, sample.z)
        hpPrevOut = (hx, hy, hz)

        let dynMag = magnitude(x: hx, y: hy, z: hz)
        waveform.append(dynMag)
        waveformXYZ.append(Sample(x: hx, y: hy, z: hz))
        dwtBuffer.append(dynMag)

        let hpOut = hrHPAlpha * (hrHPPrevOut + dynMag - hrHPPrevIn)
        hrHPPrevIn = dynMag
        hrHPPrevOut = hpOut
        let lpOut = hrLPAlpha * hpOut + (1.0 - hrLPAlpha) * hrLPPrev
        hrLPPrev = lpOut
        hrBuffer.append(lpOut)

        rmsWindow.append(dynMag)
        if rmsWindow.count > Int(fs) {
            rmsWindow.removeFirst(rmsWindow.count - Int(fs))
        }
        if sampleCount % max(1, Int(fs / 10.0)) == 0, !rmsWindow.isEmpty {
            let rv = sqrt(rmsWindow.reduce(0) { $0 + $1 * $1 } / Double(rmsWindow.count))
            rmsTrend.append(rv)
        }

        let detections = detectEvents(magnitude: dynMag, time: sample.t)
        if !detections.isEmpty, sample.t - lastEventTime > 0.01 {
            lastEventTime = sample.t
            eventTimes.append(sample.t)
            classify(detections: detections, time: sample.t, amplitude: dynMag)
        }

        return dynMag
    }

    func computeBandEnergies() {
        let data = dwtBuffer.values
        guard data.count >= 64 else {
            return
        }
        let targets = [50.0, 25.0, 12.0, 6.0, 3.0]
        for (index, target) in targets.enumerated() {
            bandEnergy[index].append(goertzelEnergy(data: data, sampleRate: fs, targetHz: target))
        }
    }

    func detectPeriodicity() {
        let data = waveform.values.suffix(Int(fs * 5))
        guard data.count >= Int(fs * 2) else {
            period = nil
            periodStd = nil
            periodCV = nil
            periodFreq = nil
            acorrRing = []
            return
        }

        let buffer = Array(data)
        let mean = buffer.reduce(0, +) / Double(buffer.count)
        let centered = buffer.map { $0 - mean }
        let variance = centered.reduce(0) { $0 + $1 * $1 }
        guard variance > 1e-20 else {
            period = nil
            periodStd = nil
            periodCV = nil
            periodFreq = nil
            acorrRing = []
            return
        }

        let minLag = max(5, Int(fs * 0.05))
        let maxLag = min(buffer.count / 2, Int(fs * 2.5))
        var acorr: [Double] = []
        for lag in minLag..<maxLag {
            var sum = 0.0
            for index in 0..<(centered.count - lag) {
                sum += centered[index] * centered[index + lag]
            }
            acorr.append(sum / variance)
        }
        acorrRing = acorr

        guard let best = acorr.enumerated().max(by: { $0.element < $1.element }) else {
            period = nil
            return
        }

        let bestLag = minLag + best.offset
        if best.element > 0.1 {
            let p = Double(bestLag) / fs
            period = p
            periodFreq = fs / Double(bestLag)
            periodCV = max(0.0, 1.0 - best.element)
            periodStd = p * (periodCV ?? 0)
        } else {
            period = nil
            periodStd = nil
            periodCV = nil
            periodFreq = nil
        }
    }

    func detectHeartbeat() {
        let minCount = Int(fs * 5)
        guard hrBuffer.values.count >= minCount else {
            hrBPM = nil
            hrConfidence = 0
            return
        }

        let buffer = Array(hrBuffer.values.suffix(Int(fs * 10)))
        let mean = buffer.reduce(0, +) / Double(buffer.count)
        let centered = buffer.map { $0 - mean }
        let variance = centered.reduce(0) { $0 + $1 * $1 }
        guard variance > 1e-20 else {
            hrBPM = nil
            hrConfidence = 0
            return
        }

        let lagLo = Int(fs * 0.3)
        let lagHi = min(Int(fs * 1.0), buffer.count / 2)
        guard lagLo < lagHi else {
            hrBPM = nil
            hrConfidence = 0
            return
        }

        var bestR = -1.0
        var bestLag = lagLo
        for lag in lagLo..<lagHi {
            var sum = 0.0
            for index in 0..<(centered.count - lag) {
                sum += centered[index] * centered[index + lag]
            }
            let r = sum / variance
            if r > bestR {
                bestR = r
                bestLag = lag
            }
        }

        if bestR > 0.15 {
            hrBPM = 60.0 / (Double(bestLag) / fs)
            hrConfidence = min(1.0, bestR)
        } else {
            hrBPM = nil
            hrConfidence = 0
        }
    }

    func currentOrientation() -> Orientation? {
        lastOrientation
    }

    private func updateOrientation(_ sample: TimedSample) {
        let dt = 1.0 / fs
        ahrs.update(
            ax: sample.x,
            ay: sample.y,
            az: sample.z,
            gxDps: gyroLatest.x,
            gyDps: gyroLatest.y,
            gzDps: gyroLatest.z,
            dt: dt
        )
        let e = ahrs.euler()
        let q = ahrs.quaternion
        lastOrientation = Orientation(roll: e.0, pitch: e.1, yaw: e.2, qw: q.0, qx: q.1, qy: q.2, qz: q.3)
    }

    private func detectEvents(magnitude mag: Double, time: Double) -> [(String, String, Double, Double)] {
        var detections: [(String, String, Double, Double)] = []
        let energy = mag * mag

        for index in 0..<3 {
            sta[index] += (energy - sta[index]) / staN[index]
            lta[index] += (energy - lta[index]) / ltaN[index]
            let ratio = sta[index] / (lta[index] + 1e-30)
            staLatest[index] = ratio
            let wasActive = staActive[index]
            if ratio > staThreshOn[index] && !wasActive {
                staActive[index] = true
                detections.append(("STA/LTA", "\(index)", ratio, mag))
            } else if ratio < staThreshOff[index] {
                staActive[index] = false
            }
        }

        if sampleCount % max(1, Int(fs / 30.0)) == 0 {
            for index in 0..<3 {
                staLtaRing[index].append(staLatest[index])
            }
        }

        cusumMu += 0.0001 * (mag - cusumMu)
        cusumPos = max(0.0, cusumPos + mag - cusumMu - cusumK)
        cusumNeg = max(0.0, cusumNeg - mag + cusumMu - cusumK)
        cusumValue = max(cusumPos, cusumNeg)
        if cusumPos > cusumH {
            detections.append(("CUSUM", "pos", cusumPos, mag))
            cusumPos = 0
        }
        if cusumNeg > cusumH {
            detections.append(("CUSUM", "neg", cusumNeg, mag))
            cusumNeg = 0
        }

        kurtBuffer.append(mag)
        if sampleCount % 10 == 0, kurtBuffer.values.count >= 50 {
            let values = kurtBuffer.values
            let mean = values.reduce(0, +) / Double(values.count)
            let m2 = values.reduce(0) { $0 + pow($1 - mean, 2) } / Double(values.count)
            let m4 = values.reduce(0) { $0 + pow($1 - mean, 4) } / Double(values.count)
            let k = m4 / (m2 * m2 + 1e-30)
            kurtosis = k
            if k > 6 {
                detections.append(("KURTOSIS", "k", k, mag))
            }
        }

        peakBuffer.append(mag)
        if peakBuffer.values.count >= 50, sampleCount % 10 == 0 {
            let sorted = peakBuffer.values.sorted()
            let n = sorted.count
            let median = sorted[n / 2]
            let mad = sorted.map { abs($0 - median) }.sorted()[n / 2]
            let sigma = 1.4826 * mad + 1e-30
            madSigma = sigma
            rms = sqrt(peakBuffer.values.reduce(0) { $0 + $1 * $1 } / Double(n))
            peak = peakBuffer.values.map { abs($0) }.max() ?? 0
            crest = peak / (rms + 1e-30)
            let deviation = abs(mag - median) / sigma
            if deviation > 8.0 {
                detections.append(("PEAK", "major", deviation, mag))
            } else if deviation > 5.0 {
                detections.append(("PEAK", "strong", deviation, mag))
            } else if deviation > 3.5 {
                detections.append(("PEAK", "medium", deviation, mag))
            } else if deviation > 2.0 {
                detections.append(("PEAK", "micro", deviation, mag))
            }
        }

        return detections
    }

    private func classify(detections: [(String, String, Double, Double)], time: Double, amplitude: Double) {
        let sources = Set(detections.map { $0.0 })
        let severity: String
        let symbol: String
        let label: String

        if sources.count >= 4 && amplitude > 0.05 {
            severity = "CHOC_MAJEUR"
            symbol = "★"
            label = "MAJOR"
        } else if sources.count >= 3 && amplitude > 0.02 {
            severity = "CHOC_MOYEN"
            symbol = "▲"
            label = "shock"
        } else if sources.contains("PEAK") && amplitude > 0.005 {
            severity = "MICRO_CHOC"
            symbol = "△"
            label = "micro-shock"
        } else if (sources.contains("STA/LTA") || sources.contains("CUSUM")) && amplitude > 0.003 {
            severity = "VIBRATION"
            symbol = "●"
            label = "vibration"
        } else if amplitude > 0.001 {
            severity = "VIB_LEGERE"
            symbol = "○"
            label = "light-vib"
        } else {
            severity = "MICRO_VIB"
            symbol = "·"
            label = "micro-vib"
        }

        var bands: [String] = []
        for (index, buffer) in bandEnergy.enumerated() where !buffer.values.isEmpty {
            let recent = Array(buffer.values.suffix(3))
            let avg = recent.reduce(0, +) / Double(recent.count)
            if avg > 1e-10 {
                bands.append(bandLabels[index].trimmingCharacters(in: .whitespaces))
            }
        }

        let formatter = DateFormatter()
        formatter.dateFormat = "HH:mm:ss.SSS"
        let display = formatter.string(from: Date(timeIntervalSince1970: time))

        events.append(
            MotionEvent(
                timestamp: time,
                displayTime: display,
                severity: severity,
                symbol: symbol,
                label: label,
                amplitude: amplitude,
                sources: Array(sources).sorted(),
                bands: bands
            )
        )
        if events.count > 500 {
            events.removeFirst(events.count - 500)
        }
    }

    private func goertzelEnergy(data: [Double], sampleRate: Double, targetHz: Double) -> Double {
        guard !data.isEmpty else { return 0 }
        let n = Double(data.count)
        let k = Int(0.5 + (n * targetHz / sampleRate))
        let omega = 2.0 * .pi * Double(k) / n
        let coeff = 2.0 * cos(omega)
        var s0 = 0.0
        var s1 = 0.0
        var s2 = 0.0
        for value in data {
            s0 = value + coeff * s1 - s2
            s2 = s1
            s1 = s0
        }
        return s1 * s1 + s2 * s2 - coeff * s1 * s2
    }
}

final class KeyboardFlashBridge {
    private let enabled: Bool
    private let binaryOverride: String?
    private let fadeMs: Int
    private let sendDt: Double
    private let runAsUser: Bool

    private var process: Process?
    private var stdinPipe: Pipe?
    private(set) var state = "off"
    private(set) var error = ""
    private(set) var runUser: String?
    private(set) var level = 0.0

    private var lastUpdate: Double?
    private var nextSend = 0.0
    private var noise = 0.0002
    private var peak = 0.0015
    private let decayPerSecond = 7.0

    init(enabled: Bool, binaryOverride: String?, fadeMs: Int = 20, sendHz: Double = 80, runAsUser: Bool = true) {
        self.enabled = enabled
        self.binaryOverride = binaryOverride
        self.fadeMs = max(0, fadeMs)
        self.sendDt = 1.0 / max(1.0, sendHz)
        self.runAsUser = runAsUser
    }

    func start() {
        guard enabled else {
            state = "disabled"
            return
        }

        guard let binary = resolveBinary() else {
            state = "missing"
            error = "set KBPULSE_BIN or build KBPulse"
            return
        }

        let process = Process()
        let pipe = Pipe()
        process.standardInput = pipe
        process.standardOutput = FileHandle.nullDevice
        process.standardError = Pipe()

        var arguments = ["--stdin-intensity", "--fade-ms", "\(fadeMs)"]
        if runAsUser,
           geteuid() == 0,
           let sudoUser = ProcessInfo.processInfo.environment["SUDO_USER"],
           sudoUser != "root" {
            process.executableURL = URL(fileURLWithPath: "/usr/bin/sudo")
            arguments = ["-u", sudoUser, binary] + arguments
            runUser = sudoUser
        } else {
            process.executableURL = URL(fileURLWithPath: binary)
        }

        process.arguments = arguments

        do {
            try process.run()
            self.process = process
            self.stdinPipe = pipe
            state = "on"
        } catch {
            state = "error"
            self.error = error.localizedDescription
        }
    }

    func update(magnitude: Double, time: Double) {
        guard state == "on", let process else { return }
        if !process.isRunning {
            state = "stopped"
            return
        }

        let dt = max(0.0, time - (lastUpdate ?? time))
        lastUpdate = time

        noise += 0.002 * (magnitude - noise)
        if magnitude > peak {
            peak += 0.15 * (magnitude - peak)
        } else {
            peak += 0.01 * (magnitude - peak)
        }

        let span = max(1e-6, peak - noise)
        let normalized = max(0.0, min(1.0, (magnitude - noise) / span))
        let pulse = pow(normalized, 0.65)
        let decay = dt > 0 ? exp(-decayPerSecond * dt) : 1.0
        level = max(pulse, level * decay)

        if time < nextSend {
            return
        }

        writeLevel(level)
        nextSend = time + sendDt
    }

    func stop() {
        writeLevel(0)
        stdinPipe?.fileHandleForWriting.closeFile()
        if let process, process.isRunning {
            process.terminate()
        }
        self.process = nil
        stdinPipe = nil
        if state == "on" {
            state = "off"
        }
    }

    func status() -> String {
        if state == "on" {
            let user = runUser ?? "root"
            return "KBPulse:on(\(user)) lvl:\(String(format: "%.2f", level)) fade:\(fadeMs)ms"
        }
        if !error.isEmpty {
            return "KBPulse:\(state) (\(error.prefix(34)))"
        }
        return "KBPulse:\(state)"
    }

    private func resolveBinary() -> String? {
        let fm = FileManager.default
        var candidates: [String?] = [binaryOverride, ProcessInfo.processInfo.environment["KBPULSE_BIN"]]
        let cwd = fm.currentDirectoryPath
        let base = cwd
        candidates.append(contentsOf: [
            "\(cwd)/KBPulse/bin/KBPulse",
            "\(base)/.localbin/KBPulse",
            "\(base)/KBPulse/build/Release/KBPulse",
            "\(base)/KBPulse/build/Debug/KBPulse",
            "\(base)/KBPulse/KBPulse",
        ])

        for candidate in candidates.compactMap({ $0 }) {
            if fm.isExecutableFile(atPath: candidate) {
                return candidate
            }
        }
        if let fromPath = which("KBPulse") {
            return fromPath
        }
        if let fromPath = which("kbpulse") {
            return fromPath
        }
        return nil
    }

    private func writeLevel(_ value: Double) {
        guard let handle = stdinPipe?.fileHandleForWriting else { return }
        if let data = String(format: "%.4f\n", value).data(using: .utf8) {
            do {
                try handle.write(contentsOf: data)
            } catch {
                state = "broken-pipe"
                self.error = error.localizedDescription
            }
        }
    }
}

func gauge(value: Double, minValue: Double, maxValue: Double, width: Int) -> String {
    let range = maxValue - minValue == 0 ? 1.0 : maxValue - minValue
    let t = Swift.max(0.0, Swift.min(1.0, (value - minValue) / range))
    let pos = Int(t * Double(width - 1))
    let center = Int(((0.0 - minValue) / range) * Double(width - 1))
    var chars = Array(repeating: "─", count: width)
    if center >= 0 && center < width {
        chars[center] = "┼"
    }
    chars[Swift.max(0, Swift.min(width - 1, pos))] = "●"
    return chars.joined()
}

func visibleLength(_ string: String) -> Int {
    var count = 0
    var inEscape = false
    for scalar in string.unicodeScalars {
        if scalar == "\u{001B}" {
            inEscape = true
            continue
        }
        if inEscape {
            if scalar == "m" {
                inEscape = false
            }
            continue
        }
        count += 1
    }
    return count
}

func line(_ content: String) -> String {
    let pad = max(0, terminalWidth - visibleLength(content))
    return "\(ANSI.dim)│\(ANSI.reset)\(content)\(String(repeating: " ", count: pad))\(ANSI.dim)│\(ANSI.reset)"
}

func separator(_ label: String = "") -> String {
    if label.isEmpty {
        return "\(ANSI.dim)├\(String(repeating: "─", count: terminalWidth))┤\(ANSI.reset)"
    }
    let rest = max(0, terminalWidth - visibleLength(label) - 1)
    return "\(ANSI.dim)├─\(label)\(String(repeating: "─", count: rest))┤\(ANSI.reset)"
}

func sparkline(_ values: [Double], width: Int, ceil: Double? = nil) -> String {
    guard !values.isEmpty else {
        return String(repeating: " ", count: width)
    }
    let data: [Double]
    if values.count < width {
        data = Array(repeating: 0.0, count: width - values.count) + values
    } else {
        data = Array(values.suffix(width))
    }
    let maxValue = max(ceil ?? data.map { abs($0) }.max() ?? 1.0, 1e-9)
    return String(data.map { value in
        let frac = min(1.0, abs(value) / maxValue)
        return sparkBlocks[min(8, Int(frac * 8.0))]
    })
}

func specRow(_ values: [Double], width: Int, floorDB: Double = -60, ceilDB: Double = -10) -> String {
    let chars = Array(" ·░▒▓█")
    guard !values.isEmpty else {
        return String(repeating: " ", count: width)
    }
    let data = values.count < width ? Array(repeating: 0.0, count: width - values.count) + values : Array(values.suffix(width))
    let range = ceilDB - floorDB
    return String(data.map { value in
        guard value > 0 else { return " " }
        let db = 10.0 * log10(value + 1e-20)
        let frac = max(0.0, min(1.0, (db - floorDB) / range))
        return chars[min(chars.count - 1, Int(frac * Double(chars.count - 1)))]
    })
}

func downsample(_ values: [Double], width: Int) -> [Double] {
    guard values.count > width else { return values }
    let step = Double(values.count) / Double(width)
    return (0..<width).map { index in
        let start = Int(Double(index) * step)
        let end = Int(Double(index + 1) * step)
        let chunk = values[start..<max(start + 1, end)]
        return chunk.max() ?? 0
    }
}

func severityColor(_ severity: String) -> String {
    switch severity {
    case "CHOC_MAJEUR":
        return ANSI.brightRed + ANSI.bold
    case "CHOC_MOYEN":
        return ANSI.red
    case "MICRO_CHOC":
        return ANSI.cyan
    case "VIBRATION":
        return ANSI.yellow
    case "VIB_LEGERE":
        return ANSI.green
    default:
        return ANSI.dim
    }
}

func lidText(_ angle: Double) -> String {
    "  \(ANSI.brightWhite)\(String(format: "%.0f", angle))°\(ANSI.reset)"
}

func alsLines(_ reading: ALSReading?, width: Int) -> [String] {
    guard let reading else {
        return ["  \(ANSI.dim)waiting for ALS data...\(ANSI.reset)", "", ""]
    }

    let channels = reading.channels.map(Double.init)
    let maxChannel = max(channels.max() ?? 1, 1)
    let normalized = channels.map { $0 / maxChannel }
    let values: [Double] = (0..<width).map { index in
        let t = Double(index) / Double(max(1, width - 1)) * Double(normalized.count - 1)
        let low = min(Int(t), normalized.count - 2)
        let frac = t - Double(low)
        return normalized[low] * (1 - frac) + normalized[low + 1] * frac
    }
    let curve = String(values.map { sparkBlocks[min(8, Int($0 * 8.99))] })
    let filled = reading.lux > 0.005 ? max(1, Int(min(1.0, reading.lux) * Double(width))) : 0
    let bar = String((0..<width).map { $0 < filled ? "█" : "·" })
    return [
        "  \(ANSI.cyan)\(curve)\(ANSI.reset)",
        "  \(ANSI.yellow)\(bar)\(ANSI.reset)  \(ANSI.brightWhite)\(String(format: "%.3f", reading.lux))\(ANSI.reset) \(ANSI.dim)lux\(ANSI.reset)",
        "  \(ANSI.dim)ch: \(reading.channels.map(String.init).joined(separator: " "))\(ANSI.reset)",
    ]
}

func render(detector: VibrationDetector, startTime: Double, kbflash: KeyboardFlashBridge?, lidAngle: Double?, alsReading: ALSReading?) -> String {
    let now = Date().timeIntervalSince1970
    let elapsed = now - startTime
    let rate = elapsed > 1 ? Double(detector.sampleCount) / elapsed : 0
    let graphWidth = terminalWidth - 4
    var lines: [String] = []
    lines.append("\(ANSI.dim)┌─\(ANSI.reset)\(ANSI.brightWhite) MOTION DETECTOR \(ANSI.reset)\(ANSI.dim)\(String(repeating: "─", count: terminalWidth - 18))┐\(ANSI.reset)")
    lines.append(line(" \(ANSI.dim)\(String(format: "%7.1fs", elapsed))\(ANSI.reset)  \(String(format: "%10d", detector.sampleCount)) smp  \(ANSI.brightWhite)\(String(format: "%.0f", rate))\(ANSI.reset) Hz  Ev:\(detector.events.count)"))

    lines.append(separator(" Waveform |a_dyn| 5s "))
    let wave = detector.waveform.values
    if !wave.isEmpty {
        let maxWave = max(wave.map(abs).max() ?? 0.0002, 0.0002)
        lines.append(line("  \(ANSI.green)\(sparkline(downsample(wave, width: graphWidth), width: graphWidth, ceil: maxWave))\(ANSI.reset)"))
        lines.append(line("  \(ANSI.dim)\(String(format: "%.5fg", maxWave))\(String(repeating: " ", count: max(1, graphWidth - 10)))0g\(ANSI.reset)"))
    } else {
        lines.append(line("  \(ANSI.dim)waiting...\(ANSI.reset)"))
        lines.append(line(""))
    }

    lines.append(separator(" Axes X / Y / Z (5s) "))
    let xyz = detector.waveformXYZ.values
    if !xyz.isEmpty {
        let xs = xyz.map(\.x)
        let ys = xyz.map(\.y)
        let zs = xyz.map(\.z)
        let maxAxis = max((xs + ys + zs).map(abs).max() ?? 0.0001, 0.0001)
        let axisWidth = graphWidth - 4
        lines.append(line("  \(ANSI.red)X\(ANSI.reset) \(sparkline(downsample(xs, width: axisWidth), width: axisWidth, ceil: maxAxis))"))
        lines.append(line("  \(ANSI.green)Y\(ANSI.reset) \(sparkline(downsample(ys, width: axisWidth), width: axisWidth, ceil: maxAxis))"))
        lines.append(line("  \(ANSI.cyan)Z\(ANSI.reset) \(sparkline(downsample(zs, width: axisWidth), width: axisWidth, ceil: maxAxis))"))
    } else {
        lines.append(line("  \(ANSI.dim)X\(ANSI.reset)"))
        lines.append(line("  \(ANSI.dim)Y\(ANSI.reset)"))
        lines.append(line("  \(ANSI.dim)Z\(ANSI.reset)"))
    }

    lines.append(separator(" Spectrum bands 5s "))
    let spectrumWidth = terminalWidth - 10
    if detector.bandEnergy.contains(where: { !$0.values.isEmpty }) {
        for index in 0..<detector.bandEnergy.count {
            lines.append(line(" \(ANSI.dim)\(detector.bandLabels[index])\(ANSI.reset) \(ANSI.cyan)\(specRow(detector.bandEnergy[index].values, width: spectrumWidth))\(ANSI.reset)"))
        }
    } else {
        lines.append(line("  \(ANSI.dim)accumulating...\(ANSI.reset)"))
        for _ in 0..<4 { lines.append(line("")) }
    }

    lines.append(separator(" RMS trend 10s "))
    if !detector.rmsTrend.values.isEmpty {
        lines.append(line("  \(ANSI.yellow)\(sparkline(detector.rmsTrend.values, width: graphWidth))\(ANSI.reset)"))
    } else {
        lines.append(line("  \(ANSI.dim)accumulating...\(ANSI.reset)"))
    }

    lines.append(separator(" Detectors "))
    let detectorWidth = 25
    let names = ["fast", "med ", "slow"]
    for index in 0..<3 {
        let sp = sparkline(detector.staLtaRing[index].values, width: detectorWidth, ceil: [6.0, 5.0, 4.0][index])
        let ratio = detector.staLatest[index]
        let threshold = [3.0, 2.5, 2.0][index]
        let marker = ratio > threshold ? "*" : " "
        let color = ratio > threshold ? ANSI.brightRed : ANSI.dim
        let extra: String
        if index == 0 {
            extra = "  K:\(String(format: "%5.1f", detector.kurtosis))  CF:\(String(format: "%5.1f", detector.crest))"
        } else if index == 1 {
            extra = "  CUSUM:\(String(format: "%8.4f", detector.cusumValue))"
        } else {
            extra = "  RMS:\(String(format: "%.5f", detector.rms))g Pk:\(String(format: "%.5f", detector.peak))g"
        }
        lines.append(line(" \(ANSI.dim)STA \(names[index])\(ANSI.reset) \(ANSI.yellow)\(sp)\(ANSI.reset) \(color)\(String(format: "%5.1f", ratio))\(marker)\(ANSI.reset)\(extra)"))
    }

    lines.append(separator(" Autocorrelation (lag 0.05-2.5s) "))
    if !detector.acorrRing.isEmpty {
        let ceil = max(0.05, detector.acorrRing.map(abs).max() ?? 0.1)
        lines.append(line("  \(ANSI.brightCyan)\(sparkline(detector.acorrRing, width: graphWidth, ceil: ceil * 1.2))\(ANSI.reset)"))
    } else {
        lines.append(line("  \(ANSI.dim)accumulating...\(ANSI.reset)"))
    }

    lines.append(separator(" Pattern "))
    if let period = detector.period, let cv = detector.periodCV, cv < 0.5 {
        let regularity = Int(max(0, min(100, (1.0 - cv) * 100)))
        lines.append(line(" Period:\(String(format: "%.3fs", period)) ±\(String(format: "%.3f", detector.periodStd ?? 0))  Freq:\(String(format: "%.2fHz", detector.periodFreq ?? 0))  Reg:\(regularity)%"))
        let symbols = detector.events.suffix(12).map(\.symbol).joined(separator: "──")
        lines.append(line(" \(ANSI.dim)──\(symbols)──\(ANSI.reset)"))
    } else {
        lines.append(line(" \(ANSI.dim)no regular pattern detected\(ANSI.reset)"))
        lines.append(line(""))
    }

    let heartbeatActive = detector.hrBPM != nil && detector.hrConfidence > 0.15
    lines.append(separator(heartbeatActive ? " Heartbeat BCG ❤ " : " Heartbeat BCG "))
    if heartbeatActive, let bpm = detector.hrBPM {
        let confidence = Int(detector.hrConfidence * 100)
        lines.append(line(" \(ANSI.brightRed)♥\(ANSI.reset) \(ANSI.brightRed)\(ANSI.bold)\(String(format: "%5.1f BPM", bpm))\(ANSI.reset)   confidence: \(confidence)%   band: 0.8-3Hz"))
        let period = 60.0 / bpm
        let beatCount = max(1, graphWidth / 3)
        let beatLine = (0..<beatCount).map { index -> String in
            let phase = ((now + Double(index) * period * 0.3).truncatingRemainder(dividingBy: period)) < (period * 0.3)
            return phase ? "\(ANSI.brightRed)♥\(ANSI.reset)─" : "\(ANSI.dim)♡\(ANSI.reset)─"
        }.joined()
        lines.append(line(" \(beatLine)"))
    } else {
        lines.append(line(" \(ANSI.dim)no heartbeat detected (rest wrists on laptop)\(ANSI.reset)"))
        lines.append(line(""))
    }

    lines.append(separator(" Orientation "))
    if let orientation = detector.currentOrientation() {
        let gaugeWidth = terminalWidth - 18
        lines.append(line(" \(ANSI.dim)Roll \(ANSI.reset) \(ANSI.cyan)\(gauge(value: orientation.roll, minValue: -180, maxValue: 180, width: gaugeWidth))\(ANSI.reset) \(String(format: "%+7.1f°", orientation.roll))"))
        lines.append(line(" \(ANSI.dim)Pitch\(ANSI.reset) \(ANSI.cyan)\(gauge(value: orientation.pitch, minValue: -90, maxValue: 90, width: gaugeWidth))\(ANSI.reset) \(String(format: "%+7.1f°", orientation.pitch))"))
        lines.append(line(" \(ANSI.dim)Yaw  \(ANSI.reset) \(ANSI.cyan)\(gauge(value: orientation.yaw, minValue: -180, maxValue: 180, width: gaugeWidth))\(ANSI.reset) \(String(format: "%+7.1f°", orientation.yaw))"))
    } else {
        lines.append(line(" \(ANSI.dim)orientation warming up...\(ANSI.reset)"))
        lines.append(line(""))
        lines.append(line(""))
    }
    lines.append(line(" \(ANSI.dim)ω: \(String(format: "%+6.2f  %+6.2f  %+6.2f", detector.gyroLatest.x, detector.gyroLatest.y, detector.gyroLatest.z)) °/s\(ANSI.reset)"))

    lines.append(separator(" Lid Angle "))
    if let lidAngle {
        lines.append(line(lidText(lidAngle)))
    } else {
        lines.append(line("  \(ANSI.dim)no lid data\(ANSI.reset)"))
    }

    lines.append(separator(" Ambient Light "))
    for alsLine in alsLines(alsReading, width: terminalWidth - 13) {
        lines.append(line(alsLine))
    }

    lines.append(separator(" Events "))
    let recentEvents = Array(detector.events.suffix(5)).reversed()
    for event in recentEvents {
        let bands = event.bands.prefix(3).joined(separator: ",")
        let bandText = bands.isEmpty ? "-" : bands
        lines.append(line(" \(ANSI.dim)\(event.displayTime)\(ANSI.reset) \(severityColor(event.severity))\(event.symbol) \(event.label.padding(toLength: 11, withPad: " ", startingAt: 0))\(ANSI.reset) \(String(format: "%.5fg", event.amplitude)) \(bandText)"))
    }
    for _ in recentEvents.count..<3 {
        lines.append(line(""))
    }

    lines.append(separator())
    lines.append(line(" X:\(String(format: "%+10.6fg", detector.latestRaw.x)) Y:\(String(format: "%+10.6fg", detector.latestRaw.y)) Z:\(String(format: "%+10.6fg", detector.latestRaw.z))  |g|:\(String(format: "%.6f", detector.latestMagnitude))"))
    if let kbflash {
        lines.append(line(" \(ANSI.dim)\(kbflash.status())\(ANSI.reset)"))
    }
    lines.append(line(" \(ANSI.dim)ctrl+c to save and quit\(ANSI.reset)"))
    lines.append("\(ANSI.dim)└\(String(repeating: "─", count: terminalWidth))┘\(ANSI.reset)")
    return lines.joined(separator: "\n")
}

func which(_ program: String) -> String? {
    let process = Process()
    let pipe = Pipe()
    process.executableURL = URL(fileURLWithPath: "/usr/bin/which")
    process.arguments = [program]
    process.standardOutput = pipe
    process.standardError = FileHandle.nullDevice
    do {
        try process.run()
        process.waitUntilExit()
        guard process.terminationStatus == 0 else { return nil }
        let data = pipe.fileHandleForReading.readDataToEndOfFile()
        let path = String(decoding: data, as: UTF8.self).trimmingCharacters(in: .whitespacesAndNewlines)
        return path.isEmpty ? nil : path
    } catch {
        return nil
    }
}

func parseArguments() -> MotionConfig {
    var config = MotionConfig()
    var iterator = CommandLine.arguments.dropFirst().makeIterator()
    while let arg = iterator.next() {
        switch arg {
        case "--no-kbpulse":
            config.useKBPulse = false
        case "--kbpulse-bin":
            config.kbpulseBinary = iterator.next()
        case "--kbpulse-as-root":
            config.kbpulseAsRoot = true
        case "--duration":
            if let value = iterator.next(), let duration = Double(value) {
                config.duration = duration
            }
        case "--sample-rate":
            if let value = iterator.next(), let sampleRate = Int(value) {
                config.sampleRate = sampleRate
            }
        default:
            break
        }
    }
    return config
}

func printUsage() {
    print("usage: sudo swift run macimu-motion-live [--no-kbpulse] [--kbpulse-bin /path/to/KBPulse] [--kbpulse-as-root] [--duration seconds] [--sample-rate hz]")
}

let config = parseArguments()
if CommandLine.arguments.contains("--help") || CommandLine.arguments.contains("-h") {
    printUsage()
    exit(0)
}

if geteuid() != 0 {
    fputs("\(ANSI.brightRed)\(ANSI.bold)[!] run with: sudo swift run macimu-motion-live\(ANSI.reset)\n", stderr)
    exit(1)
}

let stopFlag = StopFlag()
signal(SIGINT, SIG_IGN)
signal(SIGTERM, SIG_IGN)
let sigintSource = DispatchSource.makeSignalSource(signal: SIGINT, queue: .main)
let sigtermSource = DispatchSource.makeSignalSource(signal: SIGTERM, queue: .main)
sigintSource.setEventHandler { stopFlag.stop() }
sigtermSource.setEventHandler { stopFlag.stop() }
sigintSource.resume()
sigtermSource.resume()

let imu = IMU(accel: true, gyro: true, als: true, lid: true, orientation: false, sampleRate: config.sampleRate)
let detector = VibrationDetector(sampleRate: config.sampleRate)
let kbflash = KeyboardFlashBridge(
    enabled: config.useKBPulse,
    binaryOverride: config.kbpulseBinary,
    fadeMs: 20,
    sendHz: 80,
    runAsUser: !config.kbpulseAsRoot
)

do {
    try imu.start()
} catch {
    fputs("\(error.localizedDescription)\n", stderr)
    exit(1)
}

kbflash.start()
let startTime = Date().timeIntervalSince1970
var lastDraw = 0.0
var lastBands = 0.0
var lastPeriod = 0.0

FileHandle.standardOutput.write(Data((ANSI.enterAlt + ANSI.hideCursor).utf8))
defer {
    kbflash.stop()
    imu.stop()
    FileHandle.standardOutput.write(Data((ANSI.showCursor + ANSI.exitAlt + "\n").utf8))

    let formatter = DateFormatter()
    formatter.dateFormat = "yyyyMMdd_HHmmss"
    let path = "vibration_log_\(formatter.string(from: Date())).json"
    let log = EventLog(
        generated: ISO8601DateFormatter().string(from: Date()),
        totalSamples: detector.sampleCount,
        events: detector.events
    )
    if let data = try? JSONEncoder.pretty.encode(log) {
        try? data.write(to: URL(fileURLWithPath: path))
        fputs("[*] saved \(detector.events.count) events to \(path)\n", stderr)
    }
}

while !stopFlag.isStopped {
    let loopNow = Date().timeIntervalSince1970
    if let duration = config.duration, loopNow - startTime >= duration {
        break
    }

    for gyro in imu.readGyroTimed() {
        detector.processGyro(Sample(x: gyro.x, y: gyro.y, z: gyro.z))
    }

    for accel in imu.readAccelTimed() {
        let magnitude = detector.processAccel(accel)
        kbflash.update(magnitude: magnitude, time: accel.t)
    }

    if loopNow - lastBands >= 0.2 {
        detector.computeBandEnergies()
        lastBands = loopNow
    }

    if loopNow - lastPeriod >= 1.0 {
        detector.detectPeriodicity()
        detector.detectHeartbeat()
        lastPeriod = loopNow
    }

    if loopNow - lastDraw >= 0.1 {
        let frame = render(
            detector: detector,
            startTime: startTime,
            kbflash: kbflash,
            lidAngle: imu.readLid(),
            alsReading: imu.readALS()
        )
        FileHandle.standardOutput.write(Data((ANSI.clear + frame).utf8))
        lastDraw = loopNow
    }

    Thread.sleep(forTimeInterval: 0.005)
}

private extension JSONEncoder {
    static var pretty: JSONEncoder {
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        return encoder
    }
}
