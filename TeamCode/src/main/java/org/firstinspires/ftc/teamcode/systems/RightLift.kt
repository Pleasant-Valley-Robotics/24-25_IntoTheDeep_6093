package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_RIGHT
import org.firstinspires.ftc.teamcode.utility.LiftConstants.ENCODER_PER_INCH_RIGHT

class RightLift(hardwareMap: HardwareMap) : Lift(
    hardwareMap.dcMotor.get("RLift")!!.apply {
        this.zeroPowerBehavior = BRAKE
        this.direction = FORWARD
    }
) {
    override val maxLiftHeight = MAX_LIFT_HEIGHT_RIGHT
    override val encoderPerInch = ENCODER_PER_INCH_RIGHT
    override fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("right lift height", liftHeight)
    }
}