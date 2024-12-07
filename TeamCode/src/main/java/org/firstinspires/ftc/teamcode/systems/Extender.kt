package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants.MAX_EXTENSION_INCH
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants.MIN_EXTENSION_INCH

/** that thing the spintake is mounted to so we can reach into submersible */
class Extender(hardwareMap: HardwareMap) {
    private val extendMotor = hardwareMap.dcMotor.get("ExtendMotor")!!.apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.REVERSE
        this.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        this.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    val extendPosition get() = extendMotor.currentPosition / ENCODER_PER_INCH

    /**
     * allows extension within the limits defined in the code.
     * if under or over the limit, will only allow movement that restores the robot
     * within the specified upper and lower bounds.
     *
     * @param power power to extend with. `[-1, 1]`
     *
     * @see MAX_EXTENSION_INCH
     * @see MIN_EXTENSION_INCH
     */
    fun extendSafe(power: Double) {
        val inExtLimitUpper = extendPosition <= MAX_EXTENSION_INCH
        val inExtLimitLower = extendPosition >= MIN_EXTENSION_INCH
        extendMotor.power =
            if (power > 0 && inExtLimitUpper || power < 0 && inExtLimitLower) power
            else 0.0
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Extended", extendMotor.currentPosition / ENCODER_PER_INCH)
    }
}