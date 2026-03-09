// swift-tools-version: 6.0

import PackageDescription

let package = Package(
    name: "MacIMU",
    platforms: [
        .macOS(.v13),
    ],
    products: [
        .library(
            name: "MacIMU",
            targets: ["MacIMU"]
        ),
        .executable(
            name: "macimu-cli",
            targets: ["macimu-cli"]
        ),
        .executable(
            name: "macimu-motion-live",
            targets: ["macimu-motion-live"]
        ),
    ],
    targets: [
        .target(
            name: "MacIMU"
        ),
        .executableTarget(
            name: "macimu-cli",
            dependencies: ["MacIMU"]
        ),
        .executableTarget(
            name: "macimu-motion-live",
            dependencies: ["MacIMU"]
        ),
    ]
)
