package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.SPINTAKE_DODGE_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.SPINTAKE_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.SPINTAKE_LOOK_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.SPINTAKE_UP_POS

/** thing with 2 grippy wheels that hand off to the flipper */
class Spintake(hardwareMap: HardwareMap) {
    private val clawLeft = hardwareMap.crservo.get("LIntake")
    private val clawRight = hardwareMap.crservo.get("RIntake")
    private val pivotServo = hardwareMap.servo.get("Wrist").apply {
        this.position = SPINTAKE_UP_POS
    }

    enum class SpintakePivotState {
        Up,
        Down,
        Dodge,
        Look,
    }

    enum class SpintakeIntakeState {
        Suck,
        Spit,
        Off,
    }

    /**
     * moves the spintake according to `param`. maps 0 to
     * `PIVOT_UP_POS` and 1 to `PIVOT_DOWN_POS`.
     *
     * @param param input to scale. `[0, 1]`
     * @see SPINTAKE_UP_POS
     * @see SPINTAKE_DOWN_POS
     */
    fun pivotParam(param: Double) {
        pivotServo.position = SPINTAKE_UP_POS * (1 - param) + SPINTAKE_DOWN_POS * param
    }

    /**
     * moves the pivot to a specified state
     *
     * @param spintakePivotState state to move the pivot to
     * @see SPINTAKE_UP_POS
     * @see SPINTAKE_DOWN_POS
     * @see SPINTAKE_DODGE_POS
     */
    fun pivotState(spintakePivotState: SpintakePivotState) {
        pivotServo.position = when (spintakePivotState) {
            SpintakePivotState.Up -> SPINTAKE_UP_POS
            SpintakePivotState.Down -> SPINTAKE_DOWN_POS
            SpintakePivotState.Dodge -> SPINTAKE_DODGE_POS
            SpintakePivotState.Look -> SPINTAKE_LOOK_POS
        }
    }

    /**
     * sets the wheel intake state
     *
     * @param spintakeIntakeState what state the wheels should be in.
     */
    fun controlIntakeState(spintakeIntakeState: SpintakeIntakeState) {
        val power = when (spintakeIntakeState) {
            SpintakeIntakeState.Suck -> -1.0
            SpintakeIntakeState.Spit -> 1.0
            SpintakeIntakeState.Off -> 0.0
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