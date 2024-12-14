package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_IN_POS
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_OUT_POS

/** the thing on the lift that scores samples in the basket */
class Flipper(hardwareMap: HardwareMap) {
    private val flipperServo = hardwareMap.servo.get("BucketPivot").apply {
        position = FLIPPER_IN_POS
    }

    enum class FlipperState {
        In,
        Out
    }

    /**
     * moves the flipper according to `param`. maps 0 to
     * `FLIPPER_IN_POS` and 1 to `FLIPPER_OUT_POS`.
     *
     * @param param input to scale. `[0, 1]`
     * @see FLIPPER_IN_POS
     * @see FLIPPER_OUT_POS
     */
    fun pivotParam(param: Double) {
        flipperServo.position = FLIPPER_IN_POS * (1 - param) + FLIPPER_OUT_POS * param
    }

    /**
     * moves the flipper out or in
     *
     * @param state what to do with the flipper
     */
    fun pivotState(state: FlipperState) {
        flipperServo.position = when (state) {
            FlipperState.In -> FLIPPER_IN_POS
            FlipperState.Out -> FLIPPER_OUT_POS
        }
    }
}