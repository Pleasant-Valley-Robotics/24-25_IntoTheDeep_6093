package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_DOWN_POS
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants.PIVOT_UP_POS

class Spintake(hardwareMap: HardwareMap) {
    private val intakeServo = hardwareMap.crservo.get("Intake")
    private val pivotServo = hardwareMap.servo.get("Pivot").apply {
        this.position = 0.0
    }

    fun pivotTo(down: Boolean) {
        pivotServo.position = if (down) PIVOT_DOWN_POS else PIVOT_UP_POS
    }

    fun theSuckAction(suckIn: Boolean, spitOut: Boolean) {
        intakeServo.power = when {
            suckIn -> 1.0
            spitOut -> -1.0
            else -> 0.0
        }
    }

    private val intakeDirection = intakeServo.power

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Intake Power", intakeDirection)
    }
}