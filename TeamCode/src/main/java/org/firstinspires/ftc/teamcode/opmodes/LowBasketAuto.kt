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
                val driveSpeed = 0.5
                val sideSpeed = 0.5
                val turnSpeed = 0.8

                parallelWait({
                    drivebase.strafeLeft(14.0, sideSpeed)
                    drivebase.driveForward(-18.0, driveSpeed)
                }, {
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_INCH + 1)
                })

                drivebase.turnToAngle(45.0, turnSpeed)
                drivebase.driveForward(-10.0, 0.2)

                flipper.pivotTo(Flipper.FlipperState.Out)
                delay(1000) // give flipper time to extend
                flipper.pivotTo(Flipper.FlipperState.In)

                parallelWait({
                    drivebase.driveForward(14.0, driveSpeed)
                    drivebase.turnToAngle(90.0, turnSpeed)
                }, {
                    lift.moveLiftTo(0.0)
                })

                drivebase.driveForward(8.0, driveSpeed)

                spintake.pivotTo(Spintake.PivotState.Down)
                spintake.controlIntakeBool(suckIn = true, spitOut = false)
                delay(2000)

                parallelWait({
                    spintake.pivotTo(Spintake.PivotState.Up)
                    delay(1000)
                    spintake.controlIntakeBool(suckIn = false, spitOut = true)
                    delay(3000)
                    spintake.pivotTo(Spintake.PivotState.Dodge)
                    spintake.controlIntakeBool(suckIn = false, spitOut = false)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_INCH + 1)
                }, {
                    drivebase.driveForward(-10.0, driveSpeed)
                    drivebase.turnToAngle(45.0, turnSpeed)
                })

                drivebase.driveForward(-16.0, 0.2)

                flipper.pivotTo(Flipper.FlipperState.Out)
                delay(1000) // give flipper time to extend
                flipper.pivotTo(Flipper.FlipperState.In)

                drivebase.driveForward(4.0, driveSpeed)
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