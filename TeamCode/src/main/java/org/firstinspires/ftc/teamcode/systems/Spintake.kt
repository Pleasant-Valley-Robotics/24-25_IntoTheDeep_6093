package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Spintake(hardwareMap: HardwareMap) {
    private val intakeServo = hardwareMap.crservo.get("Intake")
    private val pivotServo = hardwareMap.servo.get("Pivot").apply {
        this.position = -1.0
    }

    private var pivotDown = false

    fun switchWrist() {
        pivotDown = !pivotDown
        // positive is down
        pivotServo.position = if (pivotDown) 1.0 else -1.0
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