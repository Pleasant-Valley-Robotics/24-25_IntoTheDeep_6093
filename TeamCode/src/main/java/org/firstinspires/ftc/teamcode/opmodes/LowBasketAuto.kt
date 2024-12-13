package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Flipper
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@Autonomous(name = "LowBasketAuto")
class LowBasketAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")

        val drivebase = Drivebase(hardwareMap)
        val lift = LeftLift(hardwareMap)

        lift.resetLift()

        telemetry.status("initialized motors")

        waitForStart()

        val flipper = Flipper(hardwareMap)
        val spintake = Spintake(hardwareMap)


        telemetry.status("initialized servos")


        runBlocking {
            val auto = launch {
                val driveSpeed = 0.3
                val sideSpeed = 0.3
                val turnSpeed = 0.3

                spintake.pivotTo(Spintake.PivotState.Dodge)

                drivebase.strafeLeft(-1.0, sideSpeed)
                drivebase.turnToAngle(180.0, turnSpeed)

                lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_INCH)
                drivebase.driveForward(24.0, driveSpeed)

                flipper.pivotTo(Flipper.FlipperState.Out)
                delay(1000) // give flipper time to extend
                flipper.pivotTo(Flipper.FlipperState.In)
                lift.moveLiftTo(8.0)

                drivebase.strafeLeft(-31.0, sideSpeed)
                drivebase.turnToAngle(0.0, turnSpeed)
                drivebase.driveForward(-31.0, driveSpeed)

                lift.moveLiftTo(9.2)
            }

            while (opModeIsActive() && auto.isActive) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)

                telemetry.status("Running")

                yield()
            }

            auto.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.setLiftPowerSafe(0.0, true)
        }
    }
}