package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DODGE_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_UP_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.SERVO_VEL_ENC_S

/** thing with 2 grippy wheels that hand off to the flipper */
class Spintake(hardwareMap: HardwareMap) {
    private val clawLeft = hardwareMap.crservo.get("ClawLeft")
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
    fun controlIntake(suckIn: Boolean, spitOut: Boolean) {
        val power = when {
            suckIn && spitOut -> 0.0
            suckIn -> -1.0
            spitOut -> 1.0
            else -> 0.0
        }

        // negative clawLeft out
        // positive clawRight out
        clawLeft.power = -power
        clawRight.power = power
    }
}