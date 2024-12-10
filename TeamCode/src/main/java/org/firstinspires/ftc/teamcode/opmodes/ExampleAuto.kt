package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase

@Autonomous(name = "ExampleAuto")
class ExampleAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing Drivebase")
        val drivebase = Drivebase(hardwareMap)

        telemetry.status("Initialized")
        waitForStart()

        runBlocking {
            val auto = launch {
                drivebase.turnToAngle(90.0, 0.5)
            }

            while (opModeIsActive() && auto.isActive) {
                drivebase.addTelemetry(telemetry)
                telemetry.status("Running")

                yield()
            }

            // stop motors
            drivebase.controlMotors(0.0, 0.0, 0.0)

            auto.cancelAndJoin()
        }
    }
}