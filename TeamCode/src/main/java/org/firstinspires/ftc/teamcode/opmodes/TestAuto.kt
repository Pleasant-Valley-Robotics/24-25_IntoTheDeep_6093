package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase

@Autonomous(name = "TestAuto")
class TestAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val drivebase = Drivebase(hardwareMap)

        telemetry.status("Initialized")

        waitForStart()

        runBlocking {
            val driving = launch {
                with(drivebase) {
                    strafeRight(10.0, 0.5)
                    driveForward(-10.0, 0.5)
                }
            }

            while (driving.isActive && opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                telemetry.status("Running")
                yield()
            }

            driving.cancelAndJoin()
        }

        telemetry.status("Finished")
    }
}