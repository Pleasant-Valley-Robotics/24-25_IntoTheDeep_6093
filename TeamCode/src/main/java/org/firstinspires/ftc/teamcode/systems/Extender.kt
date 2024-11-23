package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants.ENCODER_PER_INCH

class Extender(hardwareMap: HardwareMap) {
    val extendMotor = hardwareMap.dcMotor.get("ExtendMotor")!!.apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.REVERSE
        this.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    val extendPosition get() = extendMotor.currentPosition / ENCODER_PER_INCH

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Extended", extendMotor.currentPosition / ENCODER_PER_INCH)
    }
}