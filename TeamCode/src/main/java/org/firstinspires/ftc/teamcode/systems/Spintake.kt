package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DODGE_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_UP_POS

/** thing with 2 grippy wheels that hand off to the flipper */
class Spintake(hardwareMap: HardwareMap) {
    private val clawLeft = hardwareMap.crservo.get("ClawLeft")
    private val clawRight = hardwareMap.crservo.get("ClawRight")
    private val pivotServo = hardwareMap.servo.get("ClawPivot").apply {
        this.position = PIVOT_UP_POS
    }

    enum class PivotState {
        Up,
        Down,
        Dodge,
    }

    enum class IntakeState {
        Suck,
        Spit,
        Off,
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
    fun pivotState(pivotState: PivotState) {
        pivotServo.position = when (pivotState) {
            PivotState.Up -> PIVOT_UP_POS
            PivotState.Down -> PIVOT_DOWN_POS
            PivotState.Dodge -> PIVOT_DODGE_POS
        }
    }

    /**
     * sets the wheel intake state
     *
     * @param intakeState what state the wheels should be in.
     */
    fun controlIntakeState(intakeState: IntakeState) {
        val power = when (intakeState) {
            IntakeState.Suck -> -1.0
            IntakeState.Spit -> 1.0
            IntakeState.Off -> 0.0
        }

        // negative clawLeft out
        // positive clawRight out
        clawLeft.power = -power
        clawRight.power = power
    }

    /**
     * directly controls both servo motors. negative is ccw.
     *
     * @param leftPower speed of left motor. `[-1, 1]`
     * @param rightPower speed of right motor. `[-1, 1]`
     */
    fun controlIntakeDirect(leftPower: Double, rightPower: Double) {
        clawLeft.power = leftPower
        clawRight.power = rightPower
    }
}