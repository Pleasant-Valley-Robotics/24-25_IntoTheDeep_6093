package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Odometry

@Autonomous(name = "DriveAuto")
class DriveAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val odometry = Odometry(hardwareMap)
        odometry.resetOdometry()
        val drivebase = Drivebase(hardwareMap, odometry)

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