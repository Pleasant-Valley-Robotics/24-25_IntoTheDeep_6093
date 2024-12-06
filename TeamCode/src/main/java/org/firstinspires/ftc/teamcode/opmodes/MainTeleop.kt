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
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants
import org.firstinspires.ftc.teamcode.utility.LiftConstants
import org.firstinspires.ftc.teamcode.utility.SpintakeConstants

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

                    val xInput = gamepad1.left_stick_x.toDouble()
                    val yInput = -gamepad1.left_stick_y.toDouble()
                    val turnInput = gamepad1.right_stick_x.toDouble()
                    drivebase.controlMotors(xInput, yInput, turnInput)

                    val liftInput = -gamepad2.left_stick_y.toDouble()
                    lift.setLiftPowerSafe(liftInput, gamepad2.b)

                    val extendInput = -gamepad2.right_stick_y.toDouble()
                    extender.extendSafe(extendInput)

                    spintake.pivotTo(gamepad2.cross)
                    val spinOut = gamepad2.right_bumper
                    val spinIn = gamepad2.right_trigger > 0.5
                    spintake.theSuckAction(spinIn, spinOut)

                    flipper.pivotTo(gamepad2.circle)
                }
            }


            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)
                spintake.addTelemetry(telemetry)

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