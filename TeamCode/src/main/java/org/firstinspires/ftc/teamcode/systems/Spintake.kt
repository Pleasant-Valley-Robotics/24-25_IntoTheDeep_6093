package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DODGE_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_UP_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.SERVO_VEL_ENC_S

/** thing with 2 grippy wheels that hand off to the flipper */
class Spintake(hardwareMap: HardwareMap) {
    private val clawLeft = hardwareMap.crservo.get("ClawLeft").apply {
        this.direction = DcMotorSimple.Direction.REVERSE
    }
    private val clawRight = hardwareMap.crservo.get("ClawRight")
    private val pivotServo = hardwareMap.servo.get("ClawPivot").apply {
        this.position = PIVOT_UP_POS
    }

    private var pivotPos = PIVOT_UP_POS

    val pivotDown get() = pivotPos >= PIVOT_DOWN_POS

    enum class PivotState {
        Up,
        Down,
        Dodge,
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
     * moves the spintake around
     *
     * @param state what pivot should do
     * @param dt time in seconds since last call
     */
    fun pivotTo(state: PivotState, dt: Double = 0.0) {
        pivotServo.position = when (state) {
            PivotState.Up -> PIVOT_UP_POS
            PivotState.Down -> PIVOT_DOWN_POS
            PivotState.Dodge -> PIVOT_DODGE_POS
        }

        pivotPos += SERVO_VEL_ENC_S * dt * if (state == PivotState.Down) 1 else -1
        pivotPos = pivotPos.coerceIn(PIVOT_UP_POS, PIVOT_DOWN_POS)
    }

    /**
     * controls how the intake wheels spin. if both parameters are true nothing happens
     *
     * @param suckIn whether the wheels should spin inwards
     * @param spitOut whether the wheels should spin outwards
     */
    fun controlIntakeBool(suckIn: Boolean, spitOut: Boolean) {
        val power = when {
            suckIn && spitOut -> 0.0
            suckIn -> -1.0
            spitOut -> 1.0
            else -> 0.0
        }

        // negative clawLeft out
        // positive clawRight out
        clawLeft.power = power
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