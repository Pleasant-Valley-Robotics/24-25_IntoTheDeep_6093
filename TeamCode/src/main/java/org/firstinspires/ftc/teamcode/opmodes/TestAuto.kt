package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.threeten.bp.Duration

@Autonomous(name = "TestAuto")
class TestAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val drivebase = Drivebase(hardwareMap)

        telemetry.status("Initialized")

        waitForStart()

        runBlocking {
            val driving = launch {
                val driveSpeed = 0.8
                val sideSpeed = 0.8
                val turnSpeed = 0.8

                with(drivebase) {
                    strafeRight(4.0, sideSpeed)
                    turnToAngle(-15.0, turnSpeed)

                    driveForward(18.0, driveSpeed)
                    turnToAngle(45.0, turnSpeed)
                    driveForward(5.0, driveSpeed)

                    driveForward(-24.0, driveSpeed)
                    turnToAngle(90.0, turnSpeed)
                    driveForward(-32.0, driveSpeed)

                    delay(100)

                    strafeRight(10.0, sideSpeed)
                    turnToAngle(75.0, turnSpeed)
                    driveForward(49.0, driveSpeed)

                    delay(100)

                    turnToAngle(75.0, turnSpeed)
                    driveForward(-46.0, driveSpeed)

                    delay(100)

                    strafeRight(17.0, sideSpeed)

                    turnToAngle(90.0, turnSpeed)
                    driveForward(50.0, driveSpeed)

                    turnToAngle(90.0, turnSpeed)
                    driveForward(-50.0, driveSpeed)

                    delay(200)

                    // drive until at the wall
                    driveForward(42.0, driveSpeed)
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