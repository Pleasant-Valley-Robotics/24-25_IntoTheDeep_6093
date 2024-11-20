package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.LiftConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_INCH
import kotlin.math.withSign

class Lift(hardwareMap: HardwareMap) {
    val leftLiftMotor = hardwareMap.dcMotor.get("LeftLiftMotor").apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.FORWARD
    }

    val rightLiftMotor = hardwareMap.dcMotor.get("RightLiftMotor").apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.FORWARD
    }

    /** gets the current left lift height as measured by encoder, in inches */
    private val leftLiftHeight get() = leftLiftMotor.currentPosition / ENCODER_PER_INCH

    /** gets the current left lift height as measured by encoder, in inches */
    private val rightLiftHeight get() = rightLiftMotor.currentPosition / ENCODER_PER_INCH

    /**
     * tries to set the lift height
     *
     * @param inches the lift's new position, in inches. maximum of [MAX_LIFT_HEIGHT_INCH]
     * @param power how fast to move the lift. `(0, 1]`
     */
    suspend fun moveLiftTo(inches: Double, power: Double) {
        if (leftLiftHeight == inches) return

        leftLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        val targetLiftHeight = inches.coerceIn(0.0, MAX_LIFT_HEIGHT_INCH)
        leftLiftMotor.power = power.withSign(targetLiftHeight - leftLiftHeight)
        rightLiftMotor.power = power.withSign(targetLiftHeight - leftLiftHeight)

        leftLiftMotor.targetPosition = (targetLiftHeight * ENCODER_PER_INCH).toInt()
        rightLiftMotor.targetPosition = (targetLiftHeight * ENCODER_PER_INCH).toInt()

        leftLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightLiftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        while (leftLiftMotor.isBusy) yield()

        leftLiftMotor.power = 0.0
        rightLiftMotor.power = 0.0
    }

    /**
     * adds all the lift data to telemetry
     *
     * @param telemetry the telemetry object to add to
     */
    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("left lift height inch", leftLiftHeight)
        telemetry.addData("right lift height inch", rightLiftHeight)
    }
}