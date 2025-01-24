package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.Clipper
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.Pivot.PivotState
import org.firstinspires.ftc.teamcode.systems.RightLift
import org.firstinspires.ftc.teamcode.systems.Spintake

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    enum class EndEffectorState {
        Intake,
        Outtake,
        Override,
    }

    override fun runOpMode() {
        telemetry.status("initializing")
        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val leftLift = LeftLift(hardwareMap)
        val rightLift = RightLift(hardwareMap)
        val extender = Extender(hardwareMap)
        val camera = Camera(hardwareMap)
        val spintake = Spintake(hardwareMap)
        val pivot = Pivot(hardwareMap)
        val bucket = Bucket(hardwareMap)
        val clipper = Clipper(hardwareMap)

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            /**
             * implements the controls from
             * [this diagram](https://github.com/Pleasant-Valley-Robotics/24-25_IntoTheDeep_6093/blob/c044f6bc06b16190bddb8a6cdaa9e33c1754b1b0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/documentation/svgviewer-png-output(1).png?raw=true)
             */
            val endEffector = launch {
                var state = EndEffectorState.Intake
                while (isActive) {
                    val oldState = state

                    state = when {
                        gamepad2.dpad_left -> EndEffectorState.Intake
                        gamepad2.dpad_up -> EndEffectorState.Outtake
                        gamepad2.dpad_right -> EndEffectorState.Override
                        else -> state
                    }

                    if (oldState != state) {
                        val (r, g, b) = when (state) {
                            EndEffectorState.Intake -> Triple(157.0, 205.0, 73.0) // green
                            EndEffectorState.Outtake -> Triple(140.0, 142.0, 226.0) // purple
                            EndEffectorState.Override -> Triple(245.0, 39.0, 64.0) // pink-red
                        }

                        gamepad2.setLedColor(r, g, b, LED_DURATION_CONTINUOUS)
                    }

                    if (gamepad2.dpad_down) {
                        leftLift.resetLift()
                        rightLift.resetLift()
                        extender.resetExtender()
                    }

                    when (state) {
                        EndEffectorState.Intake -> {
                            val pivotInput = gamepad2.right_trigger.toDouble()
                            val extendInput = -gamepad2.left_stick_y.toDouble()

                            val clawSlide = gamepad2.right_stick_x.toDouble()
                            val clawPull = -gamepad2.right_stick_y.toDouble()

                            val clawLeft = (clawSlide + clawPull).coerceIn(-1.0..1.0)
                            val clawRight = (clawSlide - clawPull).coerceIn(-1.0..1.0)

                            // collision conditions
                            val extendedOut = extender.extendPosition > 2.0
                            val liftDown = leftLift.liftHeight < 3.0
                            val liftFullyDown = leftLift.liftHeight < 0.8
                            val cancelBucket = liftDown && !extendedOut

                            // slightly nudge left lift because of bucket collisions when retracting
                            leftLift.setLiftPowerSafe(if (liftFullyDown) 0.1 else 0.0)
                            rightLift.setLiftPowerSafe(0.0)
                            extender.extendSafe(extendInput)

                            pivot.pivotParam(pivotInput)
                            spintake.controlIntakeDirect(clawLeft, clawRight)
                            bucket.moveBucket(Bucket.BucketState.In)
                            clipper.moveClaw(Clipper.ClipperState.Open)
                        }

                        EndEffectorState.Outtake -> {
                            val bucketInput = gamepad2.left_trigger.toDouble()
                            val leftSlideInput = -gamepad2.left_stick_y.toDouble()
                            val rightSlideInput = -gamepad2.right_stick_y.toDouble()
                            val clipperInput = gamepad2.right_bumper

                            // collision condition
                            val extendedOut = extender.extendPosition > 2.0
                            val liftDown = leftLift.liftHeight < 3.0
                            val cancelBucket = liftDown && !extendedOut

                            leftLift.setLiftPowerSafe(leftSlideInput)
                            rightLift.setLiftPowerSafe(rightSlideInput)
                            extender.extendSafe(0.0)

                            pivot.movePivot(PivotState.Dodge)
                            spintake.controlIntakeDirect(leftPower = 0.0, rightPower = 0.0)
                            bucket.pivotParam(if (cancelBucket) 0.0 else bucketInput)
                            clipper.moveClaw(if (clipperInput) Clipper.ClipperState.Closed else Clipper.ClipperState.Open)
                        }

                        EndEffectorState.Override -> {
                            val bucketInput = gamepad2.left_trigger.toDouble()
                            val pivotInput = gamepad2.right_trigger.toDouble()
                            val leftSlideInput = -gamepad2.left_stick_y.toDouble()
                            val rightSlideInput = -gamepad2.right_stick_y.toDouble()
                            val extendInput = gamepad2.right_stick_x.toDouble()
                            val clipperInput = gamepad2.right_bumper

                            leftLift.setLiftPowerSafe(leftSlideInput, true)
                            rightLift.setLiftPowerSafe(rightSlideInput, true)
                            extender.extendSafe(extendInput, true)

                            pivot.pivotParam(pivotInput)
                            spintake.controlIntakeDirect(leftPower = 0.0, rightPower = 0.0)
                            bucket.pivotParam(bucketInput)
                            clipper.moveClaw(if (clipperInput) Clipper.ClipperState.Closed else Clipper.ClipperState.Open)
                        }
                    }

                    yield()
                }
            }

            val driving = launch {
                while (isActive) {
                    if (gamepad1.b) odometry.resetOdometry()
                    if (gamepad1.a) parallelRace({
                        drivebase.moveToBasket()
                    }, {
                        while (gamepad1.left_stick_y == 0f
                               && gamepad1.right_stick_y == 0f
                               && gamepad1.right_stick_x == 0f
                        ) yield()
                    })

                    val slowMode = gamepad1.right_trigger > 0.5
                    val slowdown = if (slowMode) 0.5 else 1.0

                    // the negations are because the robot uses a different coordinate system.
                    val xInput = -gamepad1.left_stick_y.toDouble() * slowdown
                    val yInput = -gamepad1.left_stick_x.toDouble() * slowdown
                    val turnInput = -gamepad1.right_stick_x.toDouble() * slowdown

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    yield()
                }
            }

            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                leftLift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)
                camera.addTelemetry(telemetry)
                odometry.addTelemetry(telemetry)
                telemetry.status("running")

                odometry.update()

                yield()
            }

            driving.cancelAndJoin()
            endEffector.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            leftLift.setLiftPowerSafe(0.0, true)
            rightLift.setLiftPowerSafe(0.0, true)
            extender.extendSafe(0.0, true)
        }
    }
}