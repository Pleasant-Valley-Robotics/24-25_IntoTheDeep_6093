package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_UP_POS

/** thing with 2 grippy wheels that hand off to the flipper */
class Spintake(hardwareMap: HardwareMap) {
    private val clawLeft = hardwareMap.crservo.get("ClawLeft")
    private val clawRight = hardwareMap.crservo.get("ClawRight")
    private val pivotServo = hardwareMap.servo.get("ClawPivot").apply {
        this.position = PIVOT_UP_POS
    }

    /**
     * moves the spintake down and up
     *
     * @param down if `true` then the pivot goes down. otherwise it goes up
     */
    fun pivotTo(down: Boolean) {
        pivotServo.position = if (down) PIVOT_DOWN_POS else PIVOT_UP_POS
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