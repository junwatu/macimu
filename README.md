# macimu

Native Swift access to the undocumented IMU and related sensors on Apple Silicon Macs.

This repository started as a Swift port of the original Python project:
[olvvier/apple-silicon-accelerometer](https://github.com/olvvier/apple-silicon-accelerometer).

The current codebase is now Swift-only, but the initial implementation and hardware reverse-engineering work came from that Python version.

This repo reads the Apple SPU HID sensors exposed through `AppleSPUHIDDevice`, including:

- accelerometer
- gyroscope
- lid angle
- ambient light

It also includes two Swift executables:

- `macimu-cli` for simple live sensor output
- `macimu-motion-live` for the full terminal dashboard with waveform, orientation, heartbeat estimation, events, and optional KBPulse integration

![demo](https://raw.githubusercontent.com/olvvier/apple-silicon-accelerometer/main/assets/demo.gif)

## project layout

- `Package.swift` - Swift Package manifest
- `Sources/MacIMU/` - reusable library target
- `Sources/macimu-cli/` - simple CLI executable
- `Sources/macimu-motion-live/` - terminal dashboard executable
- `KBPulse/` - vendored keyboard backlight helper

## quick start

```bash
git clone git@github.com:junwatu/macimu.git
cd macimu
swift build -c release
```

Run the simple CLI:

```bash
sudo .build/release/macimu-cli --duration 10
```

Run the terminal dashboard:

```bash
sudo .build/release/macimu-motion-live
```

## development

Debug build:

```bash
swift build
```

Run with SwiftPM:

```bash
sudo swift run macimu-cli --duration 10
sudo swift run macimu-motion-live
```

Useful flags:

```bash
sudo swift run macimu-cli --sample-rate 200
sudo swift run macimu-cli --als --lid
sudo swift run macimu-cli --no-orientation

sudo swift run macimu-motion-live --duration 20
sudo swift run macimu-motion-live --sample-rate 100
sudo swift run macimu-motion-live --no-kbpulse
sudo swift run macimu-motion-live --kbpulse-bin /path/to/KBPulse
```

Release binaries are written to:

- `.build/release/macimu-cli`
- `.build/release/macimu-motion-live`

## library example

```swift
import MacIMU

let imu = IMU(accel: true, gyro: true, orientation: true, sampleRate: 100)
try imu.start()
defer { imu.stop() }

if let accel = imu.latestAccel() {
    print(accel.x, accel.y, accel.z)
}

if let gyro = imu.latestGyro() {
    print(gyro.x, gyro.y, gyro.z)
}

if let orientation = imu.orientation() {
    print(orientation.roll, orientation.pitch, orientation.yaw)
}
```

## how it works

The sensor path is under `AppleSPUHIDDevice` in the IOKit registry on vendor usage page `0xFF00`.

- usage `3` is the accelerometer
- usage `9` is the gyroscope
- usage `4` is ambient light
- usage page `0x0020`, usage `138` is lid angle

The Swift implementation opens the HID devices directly with IOKit, registers async input report callbacks, parses the raw reports, and exposes higher-level sample/orientation APIs in `MacIMU`.

Orientation is computed with a Mahony AHRS filter.

## requirements

- Apple Silicon Mac with SPU HID sensors
- macOS
- Swift 6
- `sudo` to access the HID devices

Check whether the device exists:

```bash
ioreg -l -w0 | grep -A5 AppleSPUHIDDevice
```

## KBPulse

`macimu-motion-live` can drive the keyboard backlight from vibration intensity through the bundled `KBPulse` helper.

The dashboard will try these in order:

- `KBPULSE_BIN`
- `KBPulse/bin/KBPulse`
- common local build output paths
- `KBPulse` on `PATH`

## notes

- undocumented Apple SPU sensor path
- may break on future macOS releases
- requires root privileges
- not for medical use

## tested

- Apple Silicon build verified with Swift 6.2.4

## license

MIT
