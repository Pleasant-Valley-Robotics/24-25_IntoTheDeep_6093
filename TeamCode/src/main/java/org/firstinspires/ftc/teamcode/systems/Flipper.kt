package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_IN_POS
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_OUT_POS
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.SERVO_VEL_ENC_S

/** the thing on the lift that scores samples in the basket */
class Flipper(hardwareMap: HardwareMap) {
    private val flipperServo = hardwareMap.servo.get("BucketPivot").apply {
        position = FLIPPER_IN_POS
    }

    private var flipperPos = FLIPPER_IN_POS

    val flipperIn get() = flipperPos >= FLIPPER_IN_POS

    /**
     * moves the flipper out or in
     *
     * @param out if `true`, moves flipper out. otherwise, moves flipper in.
     * @param dt time since last call, used for position integration
     */
    fun pivotTo(out: Boolean, dt: Double) {
        flipperServo.position = if (out) FLIPPER_OUT_POS else FLIPPER_IN_POS
        flipperPos += SERVO_VEL_ENC_S * dt * if (out) -1 else 1
        flipperPos = flipperPos.coerceIn(FLIPPER_OUT_POS, FLIPPER_IN_POS)
    }
}