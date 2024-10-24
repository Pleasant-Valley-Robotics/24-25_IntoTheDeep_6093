package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.PrototypeConstants

class Pivot(hardwareMap: HardwareMap) {

    private val MAX_HEIGHT = PrototypeConstants.MAX_LIFT_HEIGHT

    private val pivotServo = hardwareMap.servo.get("Pivot").apply {
        this.direction = Servo.Direction.REVERSE
    }

    fun runToAngle(position: Double) {
        var targetPivot = position.coerceIn(0.0, MAX_HEIGHT)
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