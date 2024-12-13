package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utility.LiftConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_INCH
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MIN_LIFT_HEIGHT_INCH
import kotlin.math.absoluteValue
import kotlin.math.withSign

abstract class Lift(private val liftMotor: DcMotor) {
    val liftHeight get() = liftMotor.currentPosition / ENCODER_PER_INCH

    fun resetLift() {
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        liftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private var overriding = false

    /**
     * sets the lift power while respecting limits. if under or over the specified limits,
     * only movement that restores the specified bounds is allowed.
     *
     * override also allows for manual overriding of limits, and if override goes from `true` to
     * `false`, it will also reset the lift position.
     *
     * @param power the power to move the lift with. `[-1, 1]`
     * @param override whether to override the lift limits
     *
     * @see MAX_LIFT_HEIGHT_INCH
     * @see MIN_LIFT_HEIGHT_INCH
     */
    fun setLiftPowerSafe(power: Double, override: Boolean) {
        val inLiftLimitUpper = liftHeight <= MAX_LIFT_HEIGHT_INCH
        val inLiftLimitLower = liftHeight >= MIN_LIFT_HEIGHT_INCH

        liftMotor.power = when {
            override -> power
            power > 0 && inLiftLimitUpper -> power
            power < 0 && inLiftLimitLower -> power
            else -> 0.0
        }

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
            liftMotor.power = power.withSign(error)
            yield()
        } while (error.absoluteValue > threshold)

        liftMotor.power = 0.0
    }

    abstract fun addTelemetry(telemetry: Telemetry)
}