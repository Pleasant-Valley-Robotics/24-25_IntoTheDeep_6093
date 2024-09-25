package org.firstinspires.ftc.teamcode.prototypes

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Spintake(hardwareMap: HardwareMap) {
    private val liftServo = hardwareMap.servo.get("Pivot").apply {
        this.direction = Servo.Direction.REVERSE

    }
    private val intakeServo = hardwareMap.crservo.get("Intake")

    fun theSuckAction(position: Double, suckIn: Boolean, spitOut: Boolean) {
        val targetPos = position.coerceIn(0.0, 1.0)
        val targetIntake: Double = when {
            suckIn -> 1.0
            spitOut -> -1.0
            else -> 0.0
        }

        intakeServo.power = targetIntake
        liftServo.position = targetPos

    }

    private val liftPosition = liftServo.position

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Pivot Position", liftPosition)

    }
}