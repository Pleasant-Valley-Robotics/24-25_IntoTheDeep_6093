package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking

@Autonomous(name = "ExampleAuto")
class ExampleAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing Drivebase")
        val drivebase = Drivebase(hardwareMap)

        telemetry.status("Initialized")
        waitForStart()

        runBlocking {
            val auto = launch {
                coroutineScope {
                    drivebase.driveForward(10.0, 1.0)
                    drivebase.strafeRight(10.0, 1.0)
                }
                coroutineScope {
                    launch { drivebase.strafeRight(10.0, 1.0) }
                    launch { drivebase.driveForward(10.0, 1.0) }
                }
            }

            while (opModeIsActive() && auto.isActive) {
                drivebase.addTelemetry(telemetry)
                telemetry.status("Running")
            }

            auto.cancelAndJoin()
        }
    }
}