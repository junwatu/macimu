import Foundation

public func magnitude(x: Double, y: Double, z: Double) -> Double {
    sqrt(x * x + y * y + z * z)
}

private func biquadLowPassCoefficients(cutoffHz: Double, sampleRate: Double, q: Double = 0.7071) -> (Double, Double, Double, Double, Double) {
    let w0 = 2.0 * .pi * cutoffHz / sampleRate
    let alpha = sin(w0) / (2.0 * q)
    let cosW0 = cos(w0)
    let b0 = (1.0 - cosW0) / 2.0
    let b1 = 1.0 - cosW0
    let b2 = (1.0 - cosW0) / 2.0
    let a0 = 1.0 + alpha
    let a1 = -2.0 * cosW0
    let a2 = 1.0 - alpha
    return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
}

private func biquadHighPassCoefficients(cutoffHz: Double, sampleRate: Double, q: Double = 0.7071) -> (Double, Double, Double, Double, Double) {
    let w0 = 2.0 * .pi * cutoffHz / sampleRate
    let alpha = sin(w0) / (2.0 * q)
    let cosW0 = cos(w0)
    let b0 = (1.0 + cosW0) / 2.0
    let b1 = -(1.0 + cosW0)
    let b2 = (1.0 + cosW0) / 2.0
    let a0 = 1.0 + alpha
    let a1 = -2.0 * cosW0
    let a2 = 1.0 - alpha
    return (b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)
}

private func biquadFilter(samples: [Sample], coeffs: (Double, Double, Double, Double, Double)) -> [Sample] {
    guard !samples.isEmpty else {
        return []
    }

    let (b0, b1, b2, a1, a2) = coeffs
    var dx1 = 0.0
    var dx2 = 0.0
    var dy1 = 0.0
    var dy2 = 0.0
    var dz1 = 0.0
    var dz2 = 0.0
    var result: [Sample] = []
    result.reserveCapacity(samples.count)

    for sample in samples {
        let ox = b0 * sample.x + dx1
        dx1 = b1 * sample.x - a1 * ox + dx2
        dx2 = b2 * sample.x - a2 * ox

        let oy = b0 * sample.y + dy1
        dy1 = b1 * sample.y - a1 * oy + dy2
        dy2 = b2 * sample.y - a2 * oy

        let oz = b0 * sample.z + dz1
        dz1 = b1 * sample.z - a1 * oz + dz2
        dz2 = b2 * sample.z - a2 * oz

        result.append(Sample(x: ox, y: oy, z: oz))
    }

    return result
}

public final class GravityKalman: @unchecked Sendable {
    private let processNoise: Double
    private let measurementNoise: Double
    private var gx = 0.0
    private var gy = 0.0
    private var gz = -1.0
    private var px = 1.0
    private var py = 1.0
    private var pz = 1.0
    private var initialized = false

    public init(processNoise: Double = 0.001, measurementNoise: Double = 0.1) {
        self.processNoise = processNoise
        self.measurementNoise = measurementNoise
    }

    public func update(ax: Double, ay: Double, az: Double) -> Sample {
        if !initialized {
            gx = ax
            gy = ay
            gz = az
            initialized = true
            return Sample(x: gx, y: gy, z: gz)
        }

        let nextPx = px + processNoise
        let nextPy = py + processNoise
        let nextPz = pz + processNoise

        let kx = nextPx / (nextPx + measurementNoise)
        let ky = nextPy / (nextPy + measurementNoise)
        let kz = nextPz / (nextPz + measurementNoise)

        gx += kx * (ax - gx)
        gy += ky * (ay - gy)
        gz += kz * (az - gz)

        px = (1.0 - kx) * nextPx
        py = (1.0 - ky) * nextPy
        pz = (1.0 - kz) * nextPz

        return Sample(x: gx, y: gy, z: gz)
    }

    public var gravity: Sample {
        Sample(x: gx, y: gy, z: gz)
    }

    public func reset() {
        initialized = false
        px = 1.0
        py = 1.0
        pz = 1.0
    }
}

public func removeGravity(samples: [Sample], processNoise: Double = 0.001, measurementNoise: Double = 0.1) -> [Sample] {
    guard !samples.isEmpty else {
        return []
    }

    let kalman = GravityKalman(processNoise: processNoise, measurementNoise: measurementNoise)
    return samples.map { sample in
        let g = kalman.update(ax: sample.x, ay: sample.y, az: sample.z)
        return Sample(x: sample.x - g.x, y: sample.y - g.y, z: sample.z - g.z)
    }
}

public func lowPass(samples: [Sample], cutoffHz: Double, sampleRate: Double, order: Int = 2) -> [Sample] {
    guard !samples.isEmpty, cutoffHz > 0, sampleRate > 0 else {
        return samples
    }
    let coeffs = biquadLowPassCoefficients(cutoffHz: cutoffHz, sampleRate: sampleRate)
    var output = biquadFilter(samples: samples, coeffs: coeffs)
    if order >= 4 {
        output = biquadFilter(samples: output, coeffs: coeffs)
    }
    return output
}

public func highPass(samples: [Sample], cutoffHz: Double, sampleRate: Double, order: Int = 2) -> [Sample] {
    guard !samples.isEmpty, cutoffHz > 0, sampleRate > 0 else {
        return samples
    }
    let coeffs = biquadHighPassCoefficients(cutoffHz: cutoffHz, sampleRate: sampleRate)
    var output = biquadFilter(samples: samples, coeffs: coeffs)
    if order >= 4 {
        output = biquadFilter(samples: output, coeffs: coeffs)
    }
    return output
}

public func bandPass(samples: [Sample], lowHz: Double, highHz: Double, sampleRate: Double, order: Int = 2) -> [Sample] {
    lowPass(samples: highPass(samples: samples, cutoffHz: lowHz, sampleRate: sampleRate, order: order), cutoffHz: highHz, sampleRate: sampleRate, order: order)
}
