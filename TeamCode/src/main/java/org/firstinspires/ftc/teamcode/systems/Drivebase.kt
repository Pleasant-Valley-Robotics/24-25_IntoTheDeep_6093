package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utility.DriveConstants.DRIVING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DriveConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.utility.DriveConstants.STRAFING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DriveConstants.TURNING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.maxOf
import kotlin.math.absoluteValue

class Drivebase(hardwareMap: HardwareMap) {
    private val fldrive = hardwareMap.dcMotor.get("FLDrive")!!
    private val frdrive = hardwareMap.dcMotor.get("FRDrive")!!
    private val bldrive = hardwareMap.dcMotor.get("BLDrive")!!
    private val brdrive = hardwareMap.dcMotor.get("BRDrive")!!

    private val imu = hardwareMap.get(IMU::class.java, "IMU").apply {
        this.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    LogoFacingDirection.LEFT,
                    UsbFacingDirection.UP,
                )
            )
        )
    }

    private val motors = listOf(fldrive, frdrive, bldrive, brdrive)

    init {
        motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }
        motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }

        fldrive.direction = DcMotorSimple.Direction.REVERSE
        frdrive.direction = DcMotorSimple.Direction.FORWARD
        bldrive.direction = DcMotorSimple.Direction.REVERSE
        brdrive.direction = DcMotorSimple.Direction.FORWARD

        imu.resetYaw()
    }


    /**
     * function to be used in teleop, controls the motors
     *
     * @param xInput the x (strafe) input, left is negative. `[-1, 1]`
     * @param yInput the y (drive) input, forward is positive. `[-1, 1]`
     * @param turnInput the turning input, clockwise is positive. `[-1, 1]`
     */
    fun controlMotors(xInput: Double, yInput: Double, turnInput: Double) {

        // https://gm0.org/en/latest/   docs/software/tutorials/mecanum-drive.html
        val flpower = yInput + xInput + turnInput
        val frpower = yInput - xInput - turnInput
        val blpower = yInput - xInput + turnInput
        val brpower = yInput + xInput - turnInput

        val maxPower = maxOf(1.0, flpower, frpower, blpower, brpower)

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
     * @param maxPower how fast to drive. `(0, 1]`
     */
    suspend fun driveForward(inches: Double, maxPower: Double) {
        resetMotorEncoders()

        do {
            val delta = inches - yDistance

            val drivePower = (delta * DRIVING_P_GAIN).coerceIn(-maxPower, maxPower)

            controlMotors(0.0, drivePower, 0.0)

            yield()
        } while (delta.absoluteValue > 0.3)

        motors.forEach { it.power = 0.0 }
    }

    /**
     * strafes (left right movement)
     *
     * @param inches how many inches to strafe, positive is right
     * @param maxPower how fast to strafe. `(0, 1]`
     */
    suspend fun strafeRight(inches: Double, maxPower: Double) {
        resetMotorEncoders()

        do {
            val delta = inches - xDistance

            val drivePower = (delta * STRAFING_P_GAIN).coerceIn(-maxPower, maxPower)

            controlMotors(drivePower, 0.0, 0.0)

            yield()
        } while (delta.absoluteValue > 0.3)

        motors.forEach { it.power = 0.0 }
    }

    private fun resetMotorEncoders() {
        for (motor in motors) with(motor) {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    private val heading get() = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

    private val xDistance
        get() = motors
            .map { it.currentPosition }
            .zip(listOf(1, -1, -1, 1))
            .sumOf { (a, b) -> a * b } / 4 / ENCODER_PER_INCH

    private val yDistance
        get() = motors
            .map { it.currentPosition }
            .zip(listOf(1, 1, 1, 1))
            .sumOf { (a, b) -> a * b } / 4 / ENCODER_PER_INCH

    private fun wrapAngle(n: Double) = (n + 180.0).mod(360.0) - 180.0

    /**
     * Turns to an angle in degrees.
     * @param degrees Angle to turn in degrees.
     */
    suspend fun turnToAngle(degrees: Double, maxPower: Double) {
        resetMotorEncoders()

        do {
            val delta = wrapAngle(degrees - heading)

            val turnPower = (delta * TURNING_P_GAIN).coerceIn(-maxPower, maxPower)

            // if we need to turn left the error will be positive bc of coordinate system
            // so invert the turn power
            controlMotors(0.0, 0.0, -turnPower)

            yield()
        } while (delta.absoluteValue > 2)

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
        telemetry.addData("heading deg", heading)
    }
}