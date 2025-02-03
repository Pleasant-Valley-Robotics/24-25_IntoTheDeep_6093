package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.opmodes.parallelWait
import org.firstinspires.ftc.teamcode.opmodes.status
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@Autonomous(group = "Basket", preselectTeleOp = "MainTeleop")
class LowBasketAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val lift = LeftLift(hardwareMap)
        val flipper = Bucket(hardwareMap)
        val spintake = Spintake(hardwareMap)
        val pivot = Pivot(hardwareMap)

        odometry.resetOdometry()
        lift.resetLift()

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            val auto = launch {
                val driveSpeed = 0.5
                val sideSpeed = 0.5
                val turnSpeed = 0.8

                pivot.movePivot(Pivot.PivotState.Dodge)

                parallelWait({
                    drivebase.strafeLeft(14.0, sideSpeed)
                    drivebase.driveForward(-18.0, driveSpeed)
                }, {
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT + 1)
                })

                drivebase.turnToAngle(45.0, turnSpeed)
                drivebase.driveForward(-10.0, 0.2)

                flipper.moveBucket(Bucket.BucketState.Out)
                delay(1000) // give flipper time to extend
                flipper.moveBucket(Bucket.BucketState.In)

                parallelWait({
                    drivebase.driveForward(14.0, driveSpeed)
                    drivebase.turnToAngle(90.0, turnSpeed)
                }, {
                    lift.moveLiftTo(0.0)
                })

                drivebase.driveForward(8.0, driveSpeed)

                pivot.movePivot(Pivot.PivotState.Down)
                spintake.controlIntakeState(Spintake.SpintakeState.Spit)
                delay(2000)

                parallelWait({
                    pivot.movePivot(Pivot.PivotState.Up)
                    delay(1000)
                    spintake.controlIntakeState(Spintake.SpintakeState.Suck)
                    delay(3000)
                    pivot.movePivot(Pivot.PivotState.Dodge)
                    spintake.controlIntakeState(Spintake.SpintakeState.Off)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT + 1)
                }, {
                    drivebase.driveForward(-10.0, driveSpeed)
                    drivebase.turnToAngle(45.0, turnSpeed)
                })

                drivebase.driveForward(-16.0, 0.2)

                flipper.moveBucket(Bucket.BucketState.Out)
                delay(1000) // give flipper time to extend
                flipper.moveBucket(Bucket.BucketState.In)

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