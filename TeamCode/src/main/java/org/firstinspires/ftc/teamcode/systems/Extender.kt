package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Extender(hardwareMap: HardwareMap) {
    val extendMotor = hardwareMap.dcMotor.get("ExtendMotor")!!.apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.REVERSE
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Extended", extendMotor.currentPosition)
    }
}