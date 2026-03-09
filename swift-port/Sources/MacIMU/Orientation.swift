import Foundation

public final class MahonyAHRS: @unchecked Sendable {
    private var q = [1.0, 0.0, 0.0, 0.0]
    private let kp: Double
    private let ki: Double
    private var errInt = [0.0, 0.0, 0.0]
    private var initialized = false

    public init(kp: Double = 1.0, ki: Double = 0.05) {
        self.kp = kp
        self.ki = ki
    }

    public func update(
        ax: Double,
        ay: Double,
        az: Double,
        gxDps: Double,
        gyDps: Double,
        gzDps: Double,
        dt: Double
    ) {
        let aNorm = sqrt(ax * ax + ay * ay + az * az)
        guard aNorm >= 0.3 else {
            return
        }

        var gx = gxDps * .pi / 180.0
        var gy = gyDps * .pi / 180.0
        var gz = gzDps * .pi / 180.0

        if !initialized {
            let axn = ax / aNorm
            let ayn = ay / aNorm
            let azn = az / aNorm
            let pitch0 = atan2(-axn, -azn)
            let roll0 = atan2(ayn, -azn)
            let cp = cos(pitch0 * 0.5)
            let sp = sin(pitch0 * 0.5)
            let cr = cos(roll0 * 0.5)
            let sr = sin(roll0 * 0.5)
            q = [cr * cp, sr * cp, cr * sp, -sr * sp]
            initialized = true
            return
        }

        let invNorm = 1.0 / aNorm
        let axn = ax * invNorm
        let ayn = ay * invNorm
        let azn = az * invNorm

        var qw = q[0]
        var qx = q[1]
        var qy = q[2]
        var qz = q[3]

        let vx = 2.0 * (qx * qz - qw * qy)
        let vy = 2.0 * (qw * qx + qy * qz)
        let vz = qw * qw - qx * qx - qy * qy + qz * qz

        let ex = ayn * (-vz) - azn * (-vy)
        let ey = azn * (-vx) - axn * (-vz)
        let ez = axn * (-vy) - ayn * (-vx)

        errInt[0] += ki * ex * dt
        errInt[1] += ki * ey * dt
        errInt[2] += ki * ez * dt

        gx += kp * ex + errInt[0]
        gy += kp * ey + errInt[1]
        gz += kp * ez + errInt[2]

        let halfDt = 0.5 * dt
        let dw = (-qx * gx - qy * gy - qz * gz) * halfDt
        let dx = (qw * gx + qy * gz - qz * gy) * halfDt
        let dy = (qw * gy - qx * gz + qz * gx) * halfDt
        let dz = (qw * gz + qx * gy - qy * gx) * halfDt

        qw += dw
        qx += dx
        qy += dy
        qz += dz

        let n = sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if n > 0 {
            let invN = 1.0 / n
            qw *= invN
            qx *= invN
            qy *= invN
            qz *= invN
        }

        q = [qw, qx, qy, qz]
    }

    public var quaternion: (Double, Double, Double, Double) {
        (q[0], q[1], q[2], q[3])
    }

    public func euler() -> (Double, Double, Double) {
        let qw = q[0]
        let qx = q[1]
        let qy = q[2]
        let qz = q[3]

        let sinR = 2.0 * (qw * qx + qy * qz)
        let cosR = 1.0 - 2.0 * (qx * qx + qy * qy)
        let roll = atan2(sinR, cosR) * 180.0 / .pi

        let sinP = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
        let pitch = asin(sinP) * 180.0 / .pi

        let sinY = 2.0 * (qw * qz + qx * qy)
        let cosY = 1.0 - 2.0 * (qy * qy + qz * qz)
        let yaw = atan2(sinY, cosY) * 180.0 / .pi

        return (roll, pitch, yaw)
    }

    public func reset() {
        q = [1.0, 0.0, 0.0, 0.0]
        errInt = [0.0, 0.0, 0.0]
        initialized = false
    }
}
