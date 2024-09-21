package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Lift

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")
        val drivebase = Drivebase(hardwareMap)
//        val lift = Lift(hardwareMap)

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
                }
            }

            val lifting = launch {
                while (isActive) {
                    yield()
//
//                    val liftTarget = when {
//                        gamepad2.a -> 0.0
//                        gamepad2.b -> 10.0
//                        gamepad2.x -> 44.0
//                        else -> continue
//                    }
//
//                    lift.moveLiftTo(liftTarget, 1.0)
                }
            }


            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
//                lift.addTelemetry(telemetry)
                telemetry.status("running")

                yield()
            }

            driving.cancelAndJoin()
            lifting.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
        }

    }
}