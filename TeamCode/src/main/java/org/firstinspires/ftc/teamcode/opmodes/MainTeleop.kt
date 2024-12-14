package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.Flipper
import org.firstinspires.ftc.teamcode.systems.LeftLift
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
        telemetry.status("initializing motors")
        val drivebase = Drivebase(hardwareMap)
        val leftLift = LeftLift(hardwareMap)
        val rightLift = RightLift(hardwareMap)
        val extender = Extender(hardwareMap)

        telemetry.status("initialized motors")

        waitForStart()

        telemetry.status("initializing servos")
        // initialize spintake after starting to comply with ftc rules about moving before match
        val spintake = Spintake(hardwareMap)
        val flipper = Flipper(hardwareMap)

        telemetry.status("initialized servos")

        runBlocking {
            val endEffector = launch {
                var state = EndEffectorState.Intake
                while (isActive) {
                    state = when {
                        gamepad2.dpad_left -> EndEffectorState.Intake
                        gamepad2.dpad_up -> EndEffectorState.Outtake
                        gamepad2.dpad_right -> EndEffectorState.Override
                        else -> state
                    }

                    val (r, g, b) = when (state) {
                        EndEffectorState.Intake -> Triple(157.0, 205.0, 73.0)
                        EndEffectorState.Outtake -> Triple(140.0, 142.0, 226.0)
                        EndEffectorState.Override -> Triple(245.0, 39.0, 64.0)
                    }

                    gamepad2.setLedColor(r, g, b, LED_DURATION_CONTINUOUS)

                    if (gamepad2.dpad_down) {
                        leftLift.resetLift()
                        rightLift.resetLift()
                        extender.resetExtender()
                    }

                    when (state) {
                        EndEffectorState.Intake -> {
                            val bucketInput = gamepad2.left_trigger.toDouble()
                            val clawInput = gamepad2.right_trigger.toDouble()
                            val extendInput = -gamepad2.left_stick_y.toDouble()

                            val clawSlide = gamepad2.right_stick_x.toDouble()
                            val clawPull = -gamepad2.right_stick_y.toDouble()

                            val clawLeft = (clawSlide - clawPull).coerceIn(-1.0..1.0)
                            val clawRight = (clawSlide + clawPull).coerceIn(-1.0..1.0)

                            // collision conditions
                            val extendedOut = extender.extendPosition > 2.0
                            val liftDown = leftLift.liftHeight < 3.0
                            val liftFullyDown = leftLift.liftHeight < 0.4
                            val cancelBucket = liftDown && !extendedOut

                            // slightly nudge left lift because of bucket collisions when retracting
                            leftLift.setLiftPowerSafe(if (liftFullyDown) 0.1 else 0.0)
                            rightLift.setLiftPowerSafe(0.0)
                            extender.extendSafe(extendInput)

                            spintake.pivotParam(clawInput)
                            spintake.controlIntakeDirect(clawLeft, clawRight)
                            flipper.pivotParam(if (cancelBucket) 0.0 else bucketInput)
                        }

                        EndEffectorState.Outtake -> {
                            val bucketInput = gamepad2.left_trigger.toDouble()
                            val leftSlideInput = -gamepad2.left_stick_y.toDouble()
                            val rightSlideInput = -gamepad2.right_stick_y.toDouble()

                            // collision condition
                            val extendedOut = extender.extendPosition > 2.0
                            val liftDown = leftLift.liftHeight < 3.0
                            val cancelBucket = liftDown && !extendedOut

                            leftLift.setLiftPowerSafe(leftSlideInput)
                            rightLift.setLiftPowerSafe(rightSlideInput)
                            extender.extendSafe(0.0)

                            spintake.pivotState(Spintake.PivotState.Dodge)
                            spintake.controlIntakeDirect(leftPower = 0.0, rightPower = 0.0)
                            flipper.pivotParam(if (cancelBucket) 0.0 else bucketInput)
                        }

                        EndEffectorState.Override -> {
                            val bucketInput = gamepad2.left_trigger.toDouble()
                            val clawInput = gamepad2.right_trigger.toDouble()
                            val leftSlideInput = -gamepad2.left_stick_y.toDouble()
                            val rightSlideInput = -gamepad2.right_stick_y.toDouble()
                            val extendInput = gamepad2.right_stick_x.toDouble()

                            leftLift.setLiftPowerSafe(leftSlideInput, true)
                            rightLift.setLiftPowerSafe(rightSlideInput, true)
                            extender.extendSafe(extendInput, true)

                            spintake.pivotParam(clawInput)
                            spintake.controlIntakeDirect(leftPower = 0.0, rightPower = 0.0)
                            flipper.pivotParam(bucketInput)
                        }
                    }

                    yield()
                }
            }

            val driving = launch {
                while (isActive) {
                    // the negations are because the robot uses a different coordinate system.
                    val xInput = -gamepad1.left_stick_y.toDouble()
                    val yInput = -gamepad1.left_stick_x.toDouble()
                    val turnInput = -gamepad1.right_stick_x.toDouble()

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    yield()
                }
            }

            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                leftLift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)

                telemetry.status("running")

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