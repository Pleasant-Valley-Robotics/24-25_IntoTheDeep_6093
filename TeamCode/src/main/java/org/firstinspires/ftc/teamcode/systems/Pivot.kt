package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Pivot(hardwareMap: HardwareMap) {


    private val pivotServo = hardwareMap.servo.get("Pivot").apply {
        this.direction = Servo.Direction.REVERSE
    }

    fun runToAngle(position: Double) {
        var targetPivot = position.coerceIn(0.0, 1.0)
        if(position < 0.0488) {
            targetPivot = 0.0488
        }

        pivotServo.position = targetPivot
    }

    private val pivotPos = pivotServo.position

    fun addTelemetry(telem: Telemetry) {
        telem.addData("Pivot Position", pivotPos)
    }

}