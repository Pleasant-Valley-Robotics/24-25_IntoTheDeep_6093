package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.prototypes.Spintake
import org.firstinspires.ftc.teamcode.systems.Drivebase

@TeleOp(name = "MainTeleop")
class ProtoTeleop : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")
        val drivebase = Drivebase(hardwareMap)
        val spintake = Spintake(hardwareMap)

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

            val intake = launch {
                while(isActive) {
                    yield()


                    val pos = gamepad2.left_stick_y.toDouble()
                    val insideMyMouth = gamepad2.a
                    val disgustingSpecimen = gamepad2.b

                    spintake.theSuckAction(pos, insideMyMouth, disgustingSpecimen)
                }
            }


            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                spintake.addTelemetry(telemetry)
                telemetry.status("running")

                yield()
            }

            driving.cancelAndJoin()
            intake.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
        }

    }
}