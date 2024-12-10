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

    enum class FlipperState {
        In,
        Out
    }

    /**
     * moves the flipper out or in
     *
     * @param state what to do with the flipper
     * @param dt time since last call, used for position integration
     */
    fun pivotTo(state: FlipperState, dt: Double = 0.0) {
        flipperServo.position = when (state) {
            FlipperState.In -> FLIPPER_IN_POS
            FlipperState.Out -> FLIPPER_OUT_POS
        }
        flipperPos += SERVO_VEL_ENC_S * dt * if (state == FlipperState.Out) -1 else 1
        flipperPos = flipperPos.coerceIn(FLIPPER_OUT_POS, FLIPPER_IN_POS)
    }
}