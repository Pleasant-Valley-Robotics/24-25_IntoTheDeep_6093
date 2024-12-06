package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_UP_POS

class Spintake(hardwareMap: HardwareMap) {
    private val clawLeft = hardwareMap.crservo.get("ClawLeft")
    private val clawRight = hardwareMap.crservo.get("ClawRight")
    private val pivotServo = hardwareMap.servo.get("ClawPivot").apply {
        this.position = PIVOT_UP_POS
    }

    fun pivotTo(down: Boolean) {
        pivotServo.position = if (down) PIVOT_DOWN_POS else PIVOT_UP_POS
    }

    fun controlIntake(suckIn: Boolean, spitOut: Boolean) {
        val power = when {
            suckIn && spitOut -> 0.0
            suckIn -> -1.0
            spitOut -> 1.0
            else -> 0.0
        }

        // positive clawLeft out
        // negative clawRight out
        clawLeft.power = power
        clawRight.power = -power
    }
}