package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.opmodes.LAST_AUTO_START_POS
import org.firstinspires.ftc.teamcode.opmodes.RisingEdgeDetector
import org.firstinspires.ftc.teamcode.opmodes.cancelWith
import org.firstinspires.ftc.teamcode.opmodes.moveToBasket
import org.firstinspires.ftc.teamcode.opmodes.moveToRungs
import org.firstinspires.ftc.teamcode.opmodes.moveToSubLeft
import org.firstinspires.ftc.teamcode.opmodes.moveToSubRight
import org.firstinspires.ftc.teamcode.opmodes.parallelRace
import org.firstinspires.ftc.teamcode.opmodes.pickClip
import org.firstinspires.ftc.teamcode.opmodes.scoreSample
import org.firstinspires.ftc.teamcode.opmodes.status
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
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    enum class EndEffectorState {
        Intake,
        Outtake,
        Override,
    }

    val Gamepad.anyStick: Boolean
        get() = left_stick_x != 0f
                || left_stick_y != 0f
                || right_stick_x != 0f
                || right_stick_y != 0f

    override fun runOpMode() {
        telemetry.status("initializing")
        val odometry = Odometry(hardwareMap, LAST_AUTO_START_POS)
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
            var inDriveAction = false
            var inLiftAction = false
            var inArmAction = false
            var state = EndEffectorState.Intake
            val rightBumper = RisingEdgeDetector(gamepad2::right_bumper)

            val actions = launch {
                while (isActive) {
                    if (gamepad1.a) {
                        inDriveAction = true
                        cancelWith({ gamepad1.anyStick }) { moveToBasket(drivebase) }
                        inDriveAction = false
                    } else if (gamepad1.x) {
                        inDriveAction = true
                        cancelWith({ gamepad1.anyStick }) { moveToSubLeft(drivebase) }
                        inDriveAction = false
                    } else if (gamepad1.y) {
                        inDriveAction = true
                        cancelWith({ gamepad1.anyStick }) { moveToRungs(drivebase) }
                        inDriveAction = false
                    } else if (gamepad1.b) {
                        inDriveAction = true
                        cancelWith({ gamepad1.anyStick }) { moveToSubRight(drivebase) }
                        inDriveAction = false
                    } else if (state == EndEffectorState.Outtake && gamepad2.touchpad) {
                        inDriveAction = true
                        inArmAction = true
                        cancelWith({ gamepad2.anyStick }) {
                            scoreSample(
                                drivebase,
                                rightLift,
                                clipper,
                                (gamepad2.touchpad_finger_1_x + 1.0) / 2.0
                            )
                            rightBumper.set(false)
                        }
                        inDriveAction = false
                        inArmAction = false
                    } else if (state == EndEffectorState.Outtake && gamepad2.circle) {
                        inDriveAction = true
                        inArmAction = true
                        cancelWith({ gamepad2.anyStick }) {
                            pickClip(drivebase, rightLift, clipper)
                            rightBumper.set(true)
                        }
                        inDriveAction = false
                        inArmAction = false
                    } else if (state == EndEffectorState.Outtake && gamepad2.cross) {
                        state = EndEffectorState.Intake
                        inLiftAction = true
                        parallelRace({
                            delay(2500L)
                        }, {
                            cancelWith({ gamepad2.dpad_up || gamepad2.dpad_right }) {
                                leftLift.moveLiftTo(0.0)
                            }
                        })
                        leftLift.mode = DcMotor.RunMode.RUN_USING_ENCODER
                        inLiftAction = false
                    } else if (state == EndEffectorState.Intake && gamepad2.square) {
                        state = EndEffectorState.Outtake
                        inLiftAction = true
                        cancelWith({ gamepad2.anyStick }) {
                            leftLift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT - 2.0)
                        }
                        leftLift.mode = DcMotor.RunMode.RUN_USING_ENCODER
                        inLiftAction = false
                    } else if (gamepad2.left_stick_button) {
                        inArmAction = true
                        cancelWith({ gamepad2.right_stick_button }) {
                            while (true) {
                                leftLift.setLiftPowerSafe(-1.0, true)
                                rightLift.setLiftPowerSafe(-1.0, true)
                                extender.extendSafe(-1.0, false)
                                yield()
                            }
                        }
                        inArmAction = false
                    }

                    yield()
                }
            }

            /**
             * implements the controls from
             * [this diagram](https://github.com/Pleasant-Valley-Robotics/24-25_IntoTheDeep_6093/blob/c044f6bc06b16190bddb8a6cdaa9e33c1754b1b0/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/documentation/svgviewer-png-output(1).png?raw=true)
             */
            val endEffector = launch {
                while (isActive) {
                    val oldState = state

                    if (inArmAction) {
                        yield()
                        continue
                    }

                    state = when {
                        gamepad2.dpad_left -> EndEffectorState.Intake
                        gamepad2.dpad_up -> EndEffectorState.Outtake
                        gamepad2.dpad_right -> EndEffectorState.Override
                        else -> state
                    }

                    if (oldState != state) {
                        rightBumper.set(false)

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
                            val liftFullyDown = leftLift.liftHeight < 0.8

                            // slightly nudge left lift because of bucket collisions when retracting
                            if (!inLiftAction) leftLift.setLiftPowerSafe(if (liftFullyDown) 0.1 else 0.0)
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
                            val clipperInput = rightBumper.get()

                            // collision condition
                            val extendedOut = extender.extendPosition > 2.0
                            val liftDown = leftLift.liftHeight < 3.0
                            val cancelBucket = liftDown && !extendedOut

                            if (!inLiftAction) leftLift.setLiftPowerSafe(leftSlideInput)
                            rightLift.setLiftPowerSafe(rightSlideInput)
                            extender.extendSafe(0.0)

                            pivot.movePivot(PivotState.Dodge)
                            spintake.controlIntakeDirect(leftPower = 0.0, rightPower = 0.0)
                            bucket.pivotParam(if (cancelBucket) 0.0 else bucketInput)
                            clipper.moveClaw(if (clipperInput) Clipper.ClipperState.Closed else Clipper.ClipperState.Open)
                        }

                        EndEffectorState.Override -> {
                            val bucketInput = gamepad2.left_trigger.toDouble()
                            val leftSlideInput = -gamepad2.left_stick_y.toDouble()
                            val rightSlideInput = -gamepad2.right_stick_y.toDouble()
                            val extendInput = gamepad2.right_stick_x.toDouble()

                            leftLift.setLiftPowerSafe(leftSlideInput, true)
                            rightLift.setLiftPowerSafe(rightSlideInput, true)
                            extender.extendSafe(extendInput, true)

                            pivot.movePivot(PivotState.Dodge)
                            spintake.controlIntakeDirect(leftPower = 0.0, rightPower = 0.0)
                            bucket.pivotParam(bucketInput)
                        }
                    }

                    yield()
                }
            }

            val driving = launch {
                while (isActive) {
                    if (gamepad1.dpad_down) odometry.resetOdometry()
                    if (inDriveAction) {
                        yield()
                        continue
                    }

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
//                drivebase.addTelemetry(telemetry)
                leftLift.addTelemetry(telemetry)
                rightLift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)
//                camera.addTelemetry(telemetry)
                odometry.addTelemetry(telemetry)
                telemetry.addData("in drive action", inDriveAction)
                telemetry.addData("in arm action", inArmAction)
                telemetry.status("running")

                odometry.update()

                yield()
            }

            actions.cancelAndJoin()
            driving.cancelAndJoin()
            endEffector.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            leftLift.setLiftPowerSafe(0.0, true)
            rightLift.setLiftPowerSafe(0.0, true)
            extender.extendSafe(0.0, true)
        }
    }
}