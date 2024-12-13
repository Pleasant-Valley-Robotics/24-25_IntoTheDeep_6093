package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
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
import kotlin.math.absoluteValue

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
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
            var bothLifts = false
            var slideOverride = false
            var intakeOverride = false
            var spintakeDown = false
            var flipperOut = false

            val detectors = listOf(
                onRisingEdge({ gamepad2.cross }) { spintakeDown = it },
                onRisingEdge({ gamepad2.circle }) { flipperOut = it },
                onRisingEdge({ gamepad2.left_stick_button }) { bothLifts = it },
                onRisingEdge({ gamepad2.dpad_up }) { slideOverride = it },
                onRisingEdge({ gamepad2.dpad_down }) { intakeOverride = it },
            )

            val driving = launch {
                val time = ElapsedTime()
                while (isActive) {
                    yield()

                    for (update in detectors) update()

                    val dt = time.seconds()
                    time.reset()

                    // the negations are because the robot uses a different coordinate system.
                    val xInput = -gamepad1.left_stick_y.toDouble()
                    val yInput = -gamepad1.left_stick_x.toDouble()
                    val turnInput = -gamepad1.right_stick_x.toDouble()

                    val liftInput = -gamepad2.left_stick_y.toDouble()
                    val extendInput = -gamepad2.right_stick_y.toDouble()

                    val spinOut = gamepad2.right_bumper
                    val spinIn = gamepad2.right_trigger > 0.5

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    // checks a lot of cases to check whether the current requested movements
                    // will cause the spintake and flipper to collide.
                    val (dodgeSpintake, disableFlipper) = run {
                        if (intakeOverride) return@run false to false

                        val extenderOut = extender.extendPosition > 2.0

                        val liftMoving = liftInput.absoluteValue > 0.05
                        val liftInRange = leftLift.liftHeight < 3.0

                        val flipperMoveIn = !flipperOut
                        val flipperGoingIn = !flipper.flipperIn and flipperMoveIn

                        val disableFlipper = liftInRange and flipper.flipperIn
                        if (extenderOut or spintake.pivotDown) return@run false to disableFlipper

                        val dodgeLift =
                            liftInRange and liftMoving and (flipperMoveIn or flipper.flipperIn)
                        val dodgeFlipper = liftInRange and flipperGoingIn

                        (dodgeLift or dodgeFlipper) to disableFlipper
                    }

                    spintake.pivotTo(
                        state = when {
                            dodgeSpintake -> Spintake.PivotState.Dodge
                            spintakeDown -> Spintake.PivotState.Down
                            else -> Spintake.PivotState.Up
                        }, dt
                    )

                    flipper.pivotTo(
                        state = when {
                            disableFlipper -> Flipper.FlipperState.In
                            flipperOut -> Flipper.FlipperState.Out
                            else -> Flipper.FlipperState.In
                        }, dt
                    )

                    leftLift.setLiftPowerSafe(liftInput, slideOverride)

                    rightLift.setLiftPowerSafe(if (bothLifts) liftInput else 0.0, slideOverride)

                    extender.extendSafe(extendInput, slideOverride)

                    spintake.controlIntake(spinIn, spinOut)
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

            drivebase.controlMotors(0.0, 0.0, 0.0)
            leftLift.setLiftPowerSafe(0.0, true)
            extender.extendSafe(0.0, true)
        }

    }
}