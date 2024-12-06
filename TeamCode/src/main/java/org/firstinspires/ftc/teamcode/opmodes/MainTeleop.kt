package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
                while (isActive) {
                    yield()

                    // the negations are because the robot uses a different coordinate system.
                    val xInput = -gamepad1.left_stick_y.toDouble()
                    val yInput = -gamepad1.left_stick_x.toDouble()
                    val turnInput = -gamepad1.right_stick_x.toDouble()

                    val liftInput = -gamepad2.left_stick_y.toDouble()
                    val liftOverride = gamepad2.square
                    val extendInput = -gamepad2.right_stick_y.toDouble()

                    val spintakeDown = gamepad2.cross
                    val spinOut = gamepad2.right_bumper
                    val spinIn = gamepad2.right_trigger > 0.5

                    val flipperOut = gamepad2.circle

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    lift.setLiftPowerSafe(liftInput, liftOverride)

                    extender.extendSafe(extendInput)

                    spintake.pivotTo(spintakeDown)
                    spintake.controlIntake(spinIn, spinOut)

                    flipper.pivotTo(flipperOut)
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