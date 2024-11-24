package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.systems.Spintake

@Autonomous(name = "TestAuto")
class TestAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val drivebase = Drivebase(hardwareMap)
        val lift = Lift(hardwareMap)
        Spintake(hardwareMap)

        lift.resetLift()

        telemetry.status("Initialized")

        waitForStart()

        runBlocking {
            val driving = launch {
                val driveSpeed = 0.3
                val sideSpeed = 0.3
                val turnSpeed = 0.5

                drivebase.strafeRight(3.0, sideSpeed)
                drivebase.driveForward(24.0, driveSpeed)
                drivebase.driveForward(-22.0, driveSpeed)

                drivebase.turnToAngle(90.0, turnSpeed)
                drivebase.driveForward(-45.0, driveSpeed)

                delay(100)

                drivebase.strafeRight(5.0, sideSpeed)
                drivebase.turnToAngle(75.0, turnSpeed)
                drivebase.driveForward(46.0, driveSpeed)

                delay(100)

                drivebase.turnToAngle(75.0, turnSpeed)
                drivebase.driveForward(-43.0, driveSpeed)

                delay(100)

                drivebase.strafeRight(10.0, sideSpeed)

                drivebase.turnToAngle(90.0, turnSpeed)
                drivebase.driveForward(40.0, driveSpeed)

                drivebase.turnToAngle(90.0, turnSpeed)

                // facing towards net zone at wall right now

                drivebase.driveForward(-40.0, driveSpeed)
                drivebase.turnToAngle(0.0, turnSpeed)

//                lift.moveLiftTo(10.0)
//                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_INCH)

                drivebase.driveForward(-31.0, driveSpeed)
                delay(100)
//                drivebase.driveForward(-2.0, 0.1)

                lift.moveLiftTo(9.2)

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