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
                val driveSpeed = 0.5
                val turnSpeed = 0.5

                with(drivebase) {
                    strafeRight(4.0, driveSpeed)
                    turnToAngle(-15.0, turnSpeed)
                    driveForward(22.0, driveSpeed)
                    turnToAngle(45.0, turnSpeed)
                    driveForward(3.0, driveSpeed)

                    driveForward(-30.0, driveSpeed)
                    turnToAngle(90.0, turnSpeed)
                    driveForward(-27.0, driveSpeed)
                    turnToAngle(0.0, turnSpeed)
                    driveForward(5.0, driveSpeed)
                    turnToAngle(75.0, turnSpeed)
                    driveForward(50.0, driveSpeed)

                    driveForward(-46.0, driveSpeed)
                    turnToAngle(0.0, turnSpeed)

                    driveForward(13.0, driveSpeed)
                    turnToAngle(90.0, turnSpeed)
                    driveForward(42.0, driveSpeed)

                    driveForward(-4.0, driveSpeed)
                    turnToAngle(180.0, turnSpeed)
                    driveForward(112.0, driveSpeed)
                    strafeRight(12.0, driveSpeed)
                }
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