package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Winch(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.dcMotor.get("Winch")!!.apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("winch position", motor.currentPosition)
    }
}