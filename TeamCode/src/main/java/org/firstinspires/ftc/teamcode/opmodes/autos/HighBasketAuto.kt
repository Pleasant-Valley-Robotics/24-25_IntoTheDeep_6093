package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.opmodes.LAST_AUTO_START_POS
import org.firstinspires.ftc.teamcode.opmodes.moveToBasket
import org.firstinspires.ftc.teamcode.opmodes.parallelWait
import org.firstinspires.ftc.teamcode.opmodes.status
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants.MIN_EXTENSION
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@Autonomous(group = "Basket", preselectTeleOp = "MainTeleop")
class HighBasketAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(hardwareMap)

        LAST_AUTO_START_POS = null

        val drivebase = Drivebase(hardwareMap, odometry)
        val lift = LeftLift(hardwareMap)
        val extender = Extender(hardwareMap)

        val bucket = Bucket(hardwareMap)
        val spintake = Spintake(hardwareMap)
        val pivot = Pivot(hardwareMap)

        odometry.resetOdometry()
        lift.resetLift()
        extender.resetExtender()

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            val auto = launch {
                val driveSpeed = 0.5

                pivot.movePivot(Pivot.PivotState.Dodge)

                parallelWait({
                    moveToBasket(drivebase)
                }, {
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT - 2.0)
                })

                bucket.moveBucket(Bucket.BucketState.Out)
                delay(500) // give flipper time to extend
                bucket.moveBucket(Bucket.BucketState.In)

                drivebase.driveForward(4.0, driveSpeed)

                parallelWait({
                    drivebase.driveToPositionGlobal(
                        xInches = -16.16,
                        yInches = 25.66,
                        angleRadians = 1.58,
                        maxPower = 0.5,
                        precise = true,
                    )

                    pivot.movePivot(Pivot.PivotState.Down)
                    spintake.controlIntakeState(Spintake.SpintakeState.Suck)
                    delay(750)
                }, {
                    lift.moveLiftTo(0.0)
                })

                parallelWait({
                    extender.extendTo(MIN_EXTENSION, 0.5)
                }, {
                    spintake.controlIntakeState(Spintake.SpintakeState.Off)
                    pivot.movePivot(Pivot.PivotState.Up)
                    delay(500)
                    spintake.controlIntakeState(Spintake.SpintakeState.Spit)
                    delay(500)
                    pivot.movePivot(Pivot.PivotState.Dodge)
                    spintake.controlIntakeState(Spintake.SpintakeState.Off)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT - 2.0)
                }, {
                    moveToBasket(drivebase)
                })

                bucket.moveBucket(Bucket.BucketState.Out)
                delay(500)
                bucket.moveBucket(Bucket.BucketState.In)

                drivebase.driveForward(4.0, driveSpeed)

                parallelWait({
                    drivebase.driveToPositionGlobal(
                        xInches = -25.71,
                        yInches = 25.66,
                        angleRadians = 1.58,
                        maxPower = 0.5,
                        precise = true,
                    )

                    pivot.movePivot(Pivot.PivotState.Down)
                    spintake.controlIntakeState(Spintake.SpintakeState.Suck)
                    delay(750)
                }, {
                    lift.moveLiftTo(MIN_EXTENSION)
                })

                parallelWait({
                    extender.extendTo(0.0, 0.5)
                }, {
                    spintake.controlIntakeState(Spintake.SpintakeState.Off)
                    pivot.movePivot(Pivot.PivotState.Up)
                    delay(500)
                    spintake.controlIntakeState(Spintake.SpintakeState.Spit)
                    delay(500)
                    pivot.movePivot(Pivot.PivotState.Dodge)
                    spintake.controlIntakeState(Spintake.SpintakeState.Off)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT - 2.0)
                }, {
                    moveToBasket(drivebase)
                })

                bucket.moveBucket(Bucket.BucketState.Out)
                delay(500)
                bucket.moveBucket(Bucket.BucketState.In)

                drivebase.driveForward(4.0, driveSpeed)

                parallelWait({
                    drivebase.driveToPositionGlobal(
                        xInches = 65.07,
                        yInches = 27.28,
                        angleRadians = 0.0,
                        maxPower = 1.0,
                        precise = false,
                    )
                    drivebase.driveToPositionGlobal(
                        xInches = 70.92,
                        yInches = 5.03,
                        angleRadians = 0.0,
                        maxPower = 0.5,
                        precise = false,
                    )
                }, {
                    lift.moveLiftTo(0.0)
                })
            }

            while (opModeIsActive() && auto.isActive) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)
                odometry.addTelemetry(telemetry)
                telemetry.status("Running")

                odometry.update()
                yield()
            }

            auto.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.setLiftPowerSafe(0.0, true)
        }
    }
}