package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@Autonomous(name = "LowBasketAuto")
class LowBasketAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")

        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val lift = LeftLift(hardwareMap)

        lift.resetLift()

        telemetry.status("initialized motors")

        waitForStart()

        val flipper = Bucket(hardwareMap)
        val spintake = Spintake(hardwareMap)
        val pivot = Pivot(hardwareMap)


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
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT + 1)
                })

                drivebase.turnToAngle(45.0, turnSpeed)
                drivebase.driveForward(-10.0, 0.2)

                flipper.pivotState(Bucket.BucketState.Out)
                delay(1000) // give flipper time to extend
                flipper.pivotState(Bucket.BucketState.In)

                parallelWait({
                    drivebase.driveForward(14.0, driveSpeed)
                    drivebase.turnToAngle(90.0, turnSpeed)
                }, {
                    lift.moveLiftTo(0.0)
                })

                drivebase.driveForward(8.0, driveSpeed)

                pivot.pivotState(Pivot.PivotState.Down)
                spintake.controlIntakeState(Spintake.SpintakeState.Suck)
                delay(2000)

                parallelWait({
                    pivot.pivotState(Pivot.PivotState.Up)
                    delay(1000)
                    spintake.controlIntakeState(Spintake.SpintakeState.Spit)
                    delay(3000)
                    pivot.pivotState(Pivot.PivotState.Dodge)
                    spintake.controlIntakeState(Spintake.SpintakeState.Off)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT + 1)
                }, {
                    drivebase.driveForward(-10.0, driveSpeed)
                    drivebase.turnToAngle(45.0, turnSpeed)
                })

                drivebase.driveForward(-16.0, 0.2)

                flipper.pivotState(Bucket.BucketState.Out)
                delay(1000) // give flipper time to extend
                flipper.pivotState(Bucket.BucketState.In)

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