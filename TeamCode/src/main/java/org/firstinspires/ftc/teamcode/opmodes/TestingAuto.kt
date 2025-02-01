package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Odometry

@Autonomous
class TestingAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing Drivebase")
        val odometry = Odometry(hardwareMap)
        odometry.resetOdometry()
        val drivebase = Drivebase(hardwareMap, odometry)

        odometry.addTelemetry(telemetry)
        telemetry.status("Initialized")
        waitForStart()

        runBlocking {
            val auto = launch {
                drivebase.driveOffsetGlobal(
                    xInches = -23.22,
                    yInches = 7.938,
                    angleRadians = 0.8,
                    maxPower = 0.5,
                    precise = false,
                )
            }

            while (opModeIsActive() && auto.isActive) {
                drivebase.addTelemetry(telemetry)
                odometry.addTelemetry(telemetry)

                telemetry.status("Running")

                odometry.update()

                yield()
            }

            // stop motors
            drivebase.controlMotors(0.0, 0.0, 0.0)

            auto.cancelAndJoin()
        }
    }
}