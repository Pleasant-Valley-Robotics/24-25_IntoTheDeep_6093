package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class RightLift(hardwareMap: HardwareMap) : Lift(
    hardwareMap.dcMotor.get("RightLiftMotor")!!.apply {
        this.zeroPowerBehavior = BRAKE
        this.direction = FORWARD
    }
) {
    override fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("right lift height", liftHeight)
    }
}