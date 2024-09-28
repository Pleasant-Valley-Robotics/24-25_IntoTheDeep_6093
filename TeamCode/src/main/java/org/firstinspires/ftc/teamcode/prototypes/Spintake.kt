package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Spintake(hardwareMap: HardwareMap) {
    private val intakeServo = hardwareMap.crservo.get("Intake")

    fun theSuckAction(suckIn: Boolean, spitOut: Boolean) {
        val targetIntake: Double = when {
            suckIn -> 1.0
            spitOut -> -1.0
            else -> 0.0
        }

        intakeServo.power = targetIntake
        
    }

    private val intakeDirection = intakeServo.power
    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Intake Power", intakeDirection)

    }
}