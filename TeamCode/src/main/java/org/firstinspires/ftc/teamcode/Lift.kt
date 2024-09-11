package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.LiftConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.LiftConstants.MAX_LIFT_HEIGHT_INCH
import kotlin.math.withSign

class Lift(hardwareMap: HardwareMap) {
    private val liftMotor = hardwareMap.dcMotor.get("LiftMotor").apply {
        this.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        this.direction = DcMotorSimple.Direction.FORWARD
    }

    /** gets the current lift height as measured by encoder, in inches */
    private val liftHeight get() = liftMotor.currentPosition / ENCODER_PER_INCH

    /**
     * tries to set the lift height
     *
     * @param inches the lift's new position, in inches. maximum of [MAX_LIFT_HEIGHT_INCH]
     * @param power how fast to move the lift. `(0, 1]`
     */
    suspend fun moveLiftTo(inches: Double, power: Double) {
        if (liftHeight == inches) return

        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        val targetLiftHeight = inches.coerceIn(0.0, MAX_LIFT_HEIGHT_INCH)
        liftMotor.power = power.withSign(targetLiftHeight - liftHeight)
        liftMotor.targetPosition = (targetLiftHeight * ENCODER_PER_INCH).toInt()

        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION

        while (liftMotor.isBusy) yield()

        liftMotor.power = 0.0
    }

    /**
     * adds all the lift data to telemetry
     *
     * @param telemetry the telemetry object to add to
     */
    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("lift height inch", liftHeight)
    }
}