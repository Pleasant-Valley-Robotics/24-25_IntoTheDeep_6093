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
import org.firstinspires.ftc.teamcode.utility.CameraConstants.CAMERA_OFFSET_X_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.CAMERA_OFFSET_Y_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.CAMERA_RADIUS_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.SPINTAKE_DOWN_ANGLE_RAD
import org.firstinspires.ftc.teamcode.utility.CameraConstants.X_CORRECT_SPEED
import org.firstinspires.ftc.teamcode.utility.CameraConstants.X_CORRECT_THRESH_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.Y_CORRECT_MAX
import org.firstinspires.ftc.teamcode.utility.CameraConstants.Y_CORRECT_P
import org.firstinspires.ftc.teamcode.utility.CameraConstants.Y_CORRECT_THRESH_IN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.DRIVING_D_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.DRIVING_I_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.DRIVING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.ENCODER_PER_INCH
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.MOVEMENT_TOL_INCH_LOOSE
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.MOVEMENT_TOL_INCH_TIGHT
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.STRAFING_D_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.STRAFING_I_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.STRAFING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_D_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_I_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_TOL_DEG_LOOSE
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_TOL_DEG_TIGHT
import org.firstinspires.ftc.teamcode.utility.control.BangBangController
import org.firstinspires.ftc.teamcode.utility.control.ClampController
import org.firstinspires.ftc.teamcode.utility.control.PidController
import org.firstinspires.ftc.teamcode.utility.control.SqrtController
import org.firstinspires.ftc.teamcode.utility.dist
import org.firstinspires.ftc.teamcode.utility.rotate
import org.firstinspires.ftc.teamcode.utility.vision.BlockColor
import org.firstinspires.ftc.teamcode.utility.vision.PerspectiveTransform
import kotlin.math.PI
import kotlin.math.absoluteValue
import kotlin.math.cos
import kotlin.math.sin

/**
 * drivebase that contains all the code to drive our robot around.
 *
 * [uses standard coordinate frame](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
 * which means positive x is forward, positive y is left, and positive yaw is left.
 */
class Drivebase(hardwareMap: HardwareMap, private val odometry: Odometry) {
    private val fldrive = hardwareMap.dcMotor.get("FLDrive")!!
    private val frdrive = hardwareMap.dcMotor.get("FRDrive")!!
    private val bldrive = hardwareMap.dcMotor.get("BLDrive")!!
    private val brdrive = hardwareMap.dcMotor.get("BRDrive")!!

    private val imu = hardwareMap.get(IMU::class.java, "IMU").apply {
        this.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    LogoFacingDirection.UP,
                    UsbFacingDirection.LEFT,
                )
            )
        )
    }

    private val motors = listOf(fldrive, frdrive, bldrive, brdrive)

    init {
        motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }
        resetMotorEncoders()

        fldrive.direction = DcMotorSimple.Direction.REVERSE
        frdrive.direction = DcMotorSimple.Direction.FORWARD
        bldrive.direction = DcMotorSimple.Direction.REVERSE
        brdrive.direction = DcMotorSimple.Direction.FORWARD

        imu.resetYaw()
    }

    /**
     * function to be used in teleop, controls the motors
     *
     * [this link](https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html)
     * is probably useful if you don't know whats going on here.
     *
     * @param xInput the x (drive) input, forward is positive. `[-1, 1]`
     * @param yInput the y (strafe) input, left is positive. `[-1, 1]`
     * @param turnInput the turning input, ccw is positive. `[-1, 1]`
     */
    fun controlMotors(xInput: Double, yInput: Double, turnInput: Double) {
        val flpower = xInput - yInput - turnInput
        val frpower = xInput + yInput + turnInput
        val blpower = xInput + yInput - turnInput
        val brpower = xInput - yInput + turnInput

        val maxPower = listOf(1.0, flpower, frpower, blpower, brpower).max()

        fldrive.power = flpower / maxPower
        frdrive.power = frpower / maxPower
        bldrive.power = blpower / maxPower
        brdrive.power = brpower / maxPower
    }

    fun controlMotorsGlobal(xInput: Double, yInput: Double, turnInput: Double) {
        val (xLocal, yLocal) = rotate(Pair(xInput, yInput), -odometry.posRad)
        controlMotors(xLocal, yLocal, turnInput)
    }

    /**
     * resets motor encoders. note that this also sets power to zero and
     * freezes the motors for a split second
     */
    private fun resetMotorEncoders() {
        for (motor in motors) with(motor) {
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    /** heading in degrees */
    private val heading get() = imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)

    /** distance sideways in inches read from wheel encoders */
    private val yDistance
        get() = motors
                    .map { it.currentPosition }
                    // notice how it is the same as the yInput row in mecanum drive
                    .zip(listOf(-1, 1, 1, -1))
                    .sumOf { (a, b) -> a * b } / 4 / ENCODER_PER_INCH

    /** distance forwards in inches read from wheel encoders */
    private val xDistance
        get() = motors
                    .map { it.currentPosition }
                    .zip(listOf(1, 1, 1, 1))
                    .sumOf { (a, b) -> a * b } / 4 / ENCODER_PER_INCH

    /**
     * wraps an angle in degrees to the range `[-180, 180]`
     * @param n angle to wrap
     */
    private fun wrapAngle(n: Double) = (n + 180.0).mod(360.0) - 180.0
    private fun wrapRadians(n: Double) = (n + PI).mod(PI * 2) - PI

    suspend fun driveOffsetGlobal(
        xInches: Double,
        yInches: Double,
        angleRadians: Double,
        maxPower: Double,
        precise: Boolean,
    ) {
        resetMotorEncoders()

        val xControl = PidController(
            DRIVING_P_GAIN,
            DRIVING_I_GAIN,
            DRIVING_D_GAIN,
            0.2,
            maxPower,
        ) { -odometry.localVelX }
        val yControl = PidController(
            STRAFING_P_GAIN,
            STRAFING_I_GAIN,
            STRAFING_D_GAIN,
            0.2,
            maxPower,
        ) { -odometry.localVelY }
        val angControl = PidController(
            TURNING_P_GAIN,
            TURNING_I_GAIN,
            TURNING_D_GAIN,
            0.2,
            maxPower,
        ) { -odometry.velRad }

        do {
            val xError = xInches - odometry.globalPosX
            val yError = yInches - odometry.globalPosY
            val angError = wrapRadians(angleRadians - odometry.posRad)

            val (xErrorLocal, yErrorLocal) = rotate(Pair(xError, yError), -odometry.posRad)

            val xInput = xControl.accept(xErrorLocal)
            val yInput = yControl.accept(yErrorLocal)
            val angInput = angControl.accept(angError)

            controlMotors(xInput, yInput, angInput)

            yield()
        } while (
            dist(xError to yError) > (if (precise) MOVEMENT_TOL_INCH_TIGHT else MOVEMENT_TOL_INCH_LOOSE)
            || angError.absoluteValue > (if (precise) TURNING_TOL_DEG_TIGHT else TURNING_TOL_DEG_LOOSE) / 180.0 * PI
        )

        motors.forEach { it.power = 0.0 }
    }


    /**
     * drives the robot forward
     *
     * @param inches how many inches to drive, negative for backwards
     * @param maxPower how fast to drive. `(0, 1]`
     */
    suspend fun driveForward(inches: Double, maxPower: Double) {
        resetMotorEncoders()

        val controller = ClampController(DRIVING_P_GAIN, maxPower)

        controller.controlThing(
            tolerance = MOVEMENT_TOL_INCH_LOOSE,
            error = { inches - xDistance },
            output = { controlMotors(it, 0.0, 0.0) }
        )

        motors.forEach { it.power = 0.0 }
    }

    /**
     * strafes (left right movement)
     *
     * @param inches how many inches to strafe, positive is left
     * @param maxPower how fast to strafe. `(0, 1]`
     */
    suspend fun strafeLeft(inches: Double, maxPower: Double) {
        resetMotorEncoders()

        val controller = ClampController(STRAFING_P_GAIN, maxPower)

        controller.controlThing(
            tolerance = MOVEMENT_TOL_INCH_LOOSE,
            error = { inches - yDistance },
            output = { controlMotors(0.0, it, 0.0) }
        )

        motors.forEach { it.power = 0.0 }
    }

    /**
     * turns to an angle in degrees. note that zero degrees is the front of our robot,
     * and turning left is positive from there.
     *
     * @param degrees angle to turn in degrees.
     * @param maxPower maximum power to turn with. `(0, 1]`
     */
    suspend fun turnToAngle(degrees: Double, maxPower: Double) {
        resetMotorEncoders()

        val controller = SqrtController(TURNING_P_GAIN, maxPower)

        controller.controlThing(
            tolerance = TURNING_TOL_DEG_LOOSE,
            error = { wrapAngle(degrees - odometry.posRad / Math.PI * 180.0) },
            output = { controlMotors(0.0, 0.0, it) }
        )

        motors.forEach { it.power = 0.0 }
    }

    /**
     * intended to center the robot on the block, using camera feedback
     * and PID controllers.
     *
     * @param camera the camera object to use for positioning
     * @param spintake the spintake
     * @param extender the extender
     * @param color the color of the block to center on
     */
    suspend fun centerBlock(
        camera: Camera,
        spintake: Spintake,
        extender: Extender,
        color: BlockColor,
    ) {
        camera.sampleColor = color

        val xError = BangBangController(
            speed = X_CORRECT_SPEED,
            thresh = X_CORRECT_THRESH_IN,
        )

        val yError = ClampController(
            pGain = Y_CORRECT_P,
            maxControl = Y_CORRECT_MAX,
        )

        val pitch = SPINTAKE_DOWN_ANGLE_RAD
        val xOffset = CAMERA_OFFSET_X_IN
        val zOffset = CAMERA_RADIUS_IN

        val xNew = +xOffset * sin(pitch) - zOffset * cos(pitch)
        val zNew = +xOffset * cos(pitch) + zOffset * sin(pitch)

        camera.samplePipelineActive = true

        do {
            val pose = PerspectiveTransform.CameraPose(
                cameraX = xNew,
                cameraY = CAMERA_OFFSET_Y_IN,
                cameraZ = zNew,
                cameraXRot = 90.0,
                cameraYRot = pitch,
                cameraZRot = 0.0,
            )

            val (ex, ey) = camera.nearestCenterError ?: Pair(0.0, 0.0)
            val extendPower = xError.accept(ex)
            val strafePower = yError.accept(ey)

            extender.extendSafe(extendPower)
            controlMotors(0.0, strafePower, 0.0)

            yield()
        } while (ex.absoluteValue > X_CORRECT_THRESH_IN && ey.absoluteValue > Y_CORRECT_THRESH_IN)

        extender.extendSafe(0.0)
        controlMotors(0.0, 0.0, 0.0)
    }

    /**
     * adds wheel positions and heading to telemetry
     *
     * @param telemetry the telemetry object to add data to
     */
    fun addTelemetry(telemetry: Telemetry) {
        val format = "%8.2f"
        telemetry.addData("fldrive pos inch", format, fldrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("frdrive pos inch", format, frdrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("bldrive pos inch", format, bldrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("brdrive pos inch", format, brdrive.currentPosition / ENCODER_PER_INCH)
        telemetry.addData("encoder predicted x", format, xDistance)
        telemetry.addData("encoder predicted y", format, yDistance)
        telemetry.addData("heading deg", format, heading)
    }
}
