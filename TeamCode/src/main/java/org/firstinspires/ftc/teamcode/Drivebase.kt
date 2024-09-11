package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.DriveConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.DriveConstants.STRAFING_CORRECTION
import kotlin.math.roundToInt
import kotlin.math.withSign

class Drivebase(hardwareMap: HardwareMap) {
    private val fldrive = hardwareMap.dcMotor.get("FLDrive")
    private val frdrive = hardwareMap.dcMotor.get("FRDrive")
    private val bldrive = hardwareMap.dcMotor.get("BLDrive")
    private val brdrive = hardwareMap.dcMotor.get("BRDrive")

    private val motors = listOf(fldrive, frdrive, bldrive, brdrive)

    init {
        motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        fldrive.direction = DcMotorSimple.Direction.FORWARD
        frdrive.direction = DcMotorSimple.Direction.FORWARD
        bldrive.direction = DcMotorSimple.Direction.FORWARD
        brdrive.direction = DcMotorSimple.Direction.FORWARD
    }

    /**
     * function to be used in teleop, controls the motors
     *
     * @param xInput the x (strafe) input, left is negative. `[-1, 1]`
     * @param yInput the y (drive) input, forward is positive. `[-1, 1]`
     * @param turnInput the turning input, clockwise is positive. `[-1, 1]`
     */
    fun controlMotors(xInput: Double, yInput: Double, turnInput: Double) {
        motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }

        // https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
        val flpower = yInput + xInput + turnInput
        val frpower = yInput - xInput + turnInput
        val blpower = yInput - xInput - turnInput
        val brpower = yInput + xInput - turnInput

        val maxPower = maxOf(flpower, frpower, blpower, brpower)

        fldrive.power = flpower / maxPower
        frdrive.power = frpower / maxPower
        bldrive.power = blpower / maxPower
        brdrive.power = brpower / maxPower
    }

    /**
     * convenience function to work with multiple motors at once
     *
     * @param value the value to apply to the motors
     * @param signs the sign pattern to use
     * @see Signs
     */
    private inline fun <T> applyMotors(
        value: Double,
        signs: Signs,
        callback: DcMotor.(Double) -> T
    ): List<T> = signs.applySigns(value)
        .zip(motors)
        .map { callback(it.second, it.first) }

    /**
     * a sign pattern is when you address different wheels with different signs
     * on a value, such as wanting the front motors to be positive and the back
     * motors to be negative.
     *
     * these are the different sign patterns that you can address motors with.
     * note that in each of them, the front left motor will be kept the same.
     */
    enum class Signs(private val signs: List<Byte>) {
        /** normal motor pattern, keeps all signs the same */
        Normal(listOf(1, 1, 1, 1)),

        /** left side positive, right side negative */
        VSplit(listOf(1, -1, 1, -1)),

        /** front side positive, back side negative */
        HSplit(listOf(1, 1, -1, -1)),

        /** front left and back right positive, others negative */
        XSplit(listOf(1, -1, -1, 1));

        fun applySigns(value: Double) = this.signs.map { it * value }
    }


    /**
     * drives the robot forward
     *
     * @param inches how many inches to drive, negative for backwards
     * @param power how fast to drive. `(0, 1]`
     */
    suspend fun driveForward(inches: Double, power: Double) {
        applyMotors(power.withSign(inches), Signs.Normal) {
            this.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            this.power = it
            this.targetPosition = (inches * ENCODER_PER_INCH).roundToInt()
            this.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        while (motors.any { it.isBusy }) yield()

        motors.forEach { it.power = 0.0 }
    }

    /**
     * strafes (left right movement)
     *
     * @param inches how many inches to strafe, positive is right
     * @param power how fast to strafe. `(0, 1]`
     */
    suspend fun strafeRight(inches: Double, power: Double) {
        applyMotors(power.withSign(inches), Signs.XSplit) {
            this.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            this.power = it;
            this.targetPosition = (inches * ENCODER_PER_INCH * STRAFING_CORRECTION).roundToInt()
            this.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        while (motors.any { it.isBusy }) yield()

        motors.forEach { it.power = 0.0 }
    }

    /**
     * adds all this drivebase's data to a telemetry object
     *
     * @param telemetry the telemetry object to add data to
     */
    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("fldrive pos inch", fldrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("frdrive pos inch", frdrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("bldrive pos inch", bldrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("brdrive pos inch", brdrive.currentPosition / ENCODER_PER_INCH)
    }
}