import Foundation
import MacIMU

struct CLIConfig {
    var duration = 10.0
    var showOrientation = true
    var includeALS = false
    var includeLid = false
    var sampleRate: Int?
}

func parseArguments() -> CLIConfig {
    var config = CLIConfig()
    var iterator = CommandLine.arguments.dropFirst().makeIterator()

    while let arg = iterator.next() {
        switch arg {
        case "--duration":
            if let value = iterator.next(), let duration = Double(value) {
                config.duration = duration
            }
        case "--sample-rate":
            if let value = iterator.next(), let sampleRate = Int(value) {
                config.sampleRate = sampleRate
            }
        case "--no-orientation":
            config.showOrientation = false
        case "--als":
            config.includeALS = true
        case "--lid":
            config.includeLid = true
        default:
            break
        }
    }

    return config
}

func printUsage() {
    print("Usage: sudo swift run macimu-cli [--duration seconds] [--sample-rate hz] [--no-orientation] [--als] [--lid]")
}

let config = parseArguments()
if CommandLine.arguments.contains("--help") || CommandLine.arguments.contains("-h") {
    printUsage()
    exit(0)
}

let info = IMU.deviceInfo()
if !IMU.available() {
    fputs("No Apple SPU accelerometer was detected.\n", stderr)
    exit(1)
}

print("Detected sensors: \(info.sensors.joined(separator: ", "))")
if let product = info.product {
    print("Product: \(product)")
}

let imu = IMU(
    accel: true,
    gyro: true,
    als: config.includeALS,
    lid: config.includeLid,
    orientation: config.showOrientation,
    sampleRate: config.sampleRate
)

do {
    try imu.start()
} catch {
    fputs("\(error.localizedDescription)\n", stderr)
    exit(1)
}

defer { imu.stop() }

let deadline = Date().addingTimeInterval(config.duration)
while Date() < deadline {
    Thread.sleep(forTimeInterval: 0.2)

    if let accel = imu.latestAccel(), let gyro = imu.latestGyro() {
        print(
            String(
                format: "accel: %+0.3f %+0.3f %+0.3f g   gyro: %+0.3f %+0.3f %+0.3f dps",
                accel.x, accel.y, accel.z,
                gyro.x, gyro.y, gyro.z
            )
        )
    }

    if config.showOrientation, let orientation = imu.orientation() {
        print(
            String(
                format: "orientation: roll=%+0.1f pitch=%+0.1f yaw=%+0.1f",
                orientation.roll,
                orientation.pitch,
                orientation.yaw
            )
        )
    }

    if config.includeALS, let als = imu.readALS() {
        print("als: lux=\(String(format: "%.2f", als.lux)) channels=\(als.channels)")
    }

    if config.includeLid, let lid = imu.readLid() {
        print("lid: \(String(format: "%.1f", lid)) degrees")
    }
}
