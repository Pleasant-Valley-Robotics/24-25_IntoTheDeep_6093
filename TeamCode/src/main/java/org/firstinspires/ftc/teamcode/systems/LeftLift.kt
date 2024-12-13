package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class LeftLift(hardwareMap: HardwareMap) : Lift(
    hardwareMap.dcMotor.get("LeftLiftMotor")!!.apply {
        this.zeroPowerBehavior = BRAKE
        this.direction = REVERSE
    }
) {
    override fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("left lift height", liftHeight)
    }
}