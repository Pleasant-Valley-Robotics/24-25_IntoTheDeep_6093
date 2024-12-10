package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase

@Autonomous(name = "DriveAuto")
class DriveAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val drivebase = Drivebase(hardwareMap)

        telemetry.status("Initialized")

        waitForStart()

        runBlocking {
            val auto = launch {
                delay(25000)
                drivebase.driveForward(48.0, 0.5)
            }

            while (auto.isActive && opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                telemetry.status("Running")
                yield()
            }

            drivebase.controlMotors(0.0, 0.0, 0.0)
            auto.cancelAndJoin()
        }
    }
}