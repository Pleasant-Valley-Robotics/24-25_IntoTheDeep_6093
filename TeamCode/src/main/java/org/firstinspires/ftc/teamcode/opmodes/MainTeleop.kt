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
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.systems.Spintake

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")
        val drivebase = Drivebase(hardwareMap)
        val lift = Lift(hardwareMap)
        val extender = Extender(hardwareMap)
        val spintake = Spintake(hardwareMap)
        val flipper = Flipper(hardwareMap)

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            val driving = launch {
                val time = ElapsedTime()
                while (isActive) {
                    yield()

                    val dt = time.seconds()
                    time.reset()

                    // the negations are because the robot uses a different coordinate system.
                    val xInput = -gamepad1.left_stick_y.toDouble()
                    val yInput = -gamepad1.left_stick_x.toDouble()
                    val turnInput = -gamepad1.right_stick_x.toDouble()

                    val liftInput = -gamepad2.left_stick_y.toDouble()
                    val liftOverride = gamepad2.square
                    val extendInput = -gamepad2.right_stick_y.toDouble()

                    val spintakeMoveDown = gamepad2.cross
                    val spinOut = gamepad2.right_bumper
                    val spinIn = gamepad2.right_trigger > 0.5

                    val flipperMoveOut = gamepad2.circle

                    val pivotOverride = gamepad2.triangle

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    // checks a lot of cases to check whether the current requested movements
                    // will cause the spintake and flipper to collide.
                    val (dodgeSpintake, disableFlipper) = run {
                        if (pivotOverride) return@run false to false

                        val extenderOut = extender.extendPosition > 2.0

                        // "going" is false if we're already there
                        val liftMoveDown = liftInput < 0.0
                        val liftMoveUp = liftInput > 0.0
                        val liftGoingDown = !lift.liftDown and liftMoveDown

                        val flipperMoveIn = !flipperMoveOut
                        val flipperGoingIn = !flipper.flipperIn and flipperMoveIn

                        val disableFlipper = lift.liftDown and flipper.flipperIn
                        if (extenderOut or spintake.pivotDown) return@run false to disableFlipper

                        val dodgeLiftDown = liftGoingDown and (flipperMoveIn or flipper.flipperIn)
                        val dodgeLiftUp = liftMoveUp and flipper.flipperIn
                        val dodgeFlipper = lift.liftDown and flipperGoingIn

                        (dodgeLiftDown or dodgeLiftUp or dodgeFlipper) to disableFlipper
                    }

                    spintake.pivotTo(down = if (dodgeSpintake) true else spintakeMoveDown, dt)
                    flipper.pivotTo(out = if (disableFlipper) false else flipperMoveOut, dt)

                    lift.setLiftPowerSafe(liftInput, liftOverride)

                    extender.extendSafe(extendInput)

                    spintake.controlIntake(spinIn, spinOut)

                }
            }


            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)

                telemetry.status("running")

                yield()
            }

            driving.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.setLiftPowerSafe(0.0, true)
            extender.extendSafe(0.0)
        }

    }
}