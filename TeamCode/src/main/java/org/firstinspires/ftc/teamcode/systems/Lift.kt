package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.LiftConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_INCH
import kotlin.math.absoluteValue
import kotlin.math.withSign

class Lift(hardwareMap: HardwareMap) {
    private val leftLiftMotor = hardwareMap.dcMotor.get("LeftLiftMotor")!!.apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.REVERSE
    }

    private val rightLiftMotor = hardwareMap.dcMotor.get("RightLiftMotor")!!.apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.FORWARD
    }

    /** gets the current left lift height as measured by encoder, in inches */
    private val leftLiftHeight get() = leftLiftMotor.currentPosition / ENCODER_PER_INCH

    /** gets the current left lift height as measured by encoder, in inches */
    private val rightLiftHeight get() = rightLiftMotor.currentPosition / ENCODER_PER_INCH

    val liftHeight get() = (leftLiftHeight + rightLiftHeight) / 2

    private var liftPower: Double
        get() = (leftLiftMotor.power + rightLiftMotor.power) / 2
        set(value) {
            leftLiftMotor.power = value
            rightLiftMotor.power = value
        }

    fun resetLift() {
        leftLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftLiftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightLiftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private var overriding = false

    fun setLiftPowerSafe(power: Double, override: Boolean) {
        val inLiftLimitUpper = liftHeight <= MAX_LIFT_HEIGHT_INCH
        val inLiftLimitLower = liftHeight >= 0.5

        liftPower =
            if (override) power
            else if (power > 0 && inLiftLimitUpper || power < 0 && inLiftLimitLower) power
            else 0.0

        if (overriding && !override) {
            resetLift()
        }

        overriding = override
    }


    /**
     * tries to set the lift height
     *
     * @param inches the lift's new position, in inches. maximum of [MAX_LIFT_HEIGHT_INCH]
     */
    suspend fun moveLiftTo(inches: Double) {
        val threshold = 1.0
        val power = 0.5

        do {
            val error = inches - liftHeight
            liftPower = power.withSign(error)
            yield()
        } while (error.absoluteValue > threshold)

        liftPower = 0.0
    }

    /**
     * adds all the lift data to telemetry
     *
     * @param telemetry the telemetry object to add to
     */
    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("left lift height inch", leftLiftHeight)
        telemetry.addData("right lift height inch", rightLiftHeight)
        telemetry.addData("lift height", liftHeight)
    }
}