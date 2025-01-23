package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.PivotConstants.PIVOT_DODGE_POS
import org.firstinspires.ftc.teamcode.utility.PivotConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.PivotConstants.PIVOT_LOOK_POS
import org.firstinspires.ftc.teamcode.utility.PivotConstants.PIVOT_UP_POS

class Pivot(hardwareMap: HardwareMap) {
    private val pivotServo = hardwareMap.servo.get("Wrist")

    enum class PivotState {
        Up,
        Down,
        Dodge,
        Look,
    }

    /**
     * moves the spintake according to `param`. maps 0 to
     * `PIVOT_UP_POS` and 1 to `PIVOT_DOWN_POS`.
     *
     * @param param input to scale. `[0, 1]`
     * @see PIVOT_UP_POS
     * @see PIVOT_DOWN_POS
     */
    fun pivotParam(param: Double) {
        pivotServo.position = PIVOT_UP_POS * (1 - param) + PIVOT_DOWN_POS * param
    }

    /**
     * moves the pivot to a specified state
     *
     * @param pivotState state to move the pivot to
     * @see PIVOT_UP_POS
     * @see PIVOT_DOWN_POS
     * @see PIVOT_DODGE_POS
     */
    fun movePivot(pivotState: PivotState) {
        pivotServo.position = when (pivotState) {
            PivotState.Up -> PIVOT_UP_POS
            PivotState.Down -> PIVOT_DOWN_POS
            PivotState.Dodge -> PIVOT_DODGE_POS
            PivotState.Look -> PIVOT_LOOK_POS
        }
    }
}