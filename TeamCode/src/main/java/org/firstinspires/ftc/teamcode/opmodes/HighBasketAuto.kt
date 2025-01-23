package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@Autonomous(name = "HighBasketAuto")
class HighBasketAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")

        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val lift = LeftLift(hardwareMap)
        val extender = Extender(hardwareMap)

        lift.resetLift()

        telemetry.status("initialized motors")

        waitForStart()

        val flipper = Bucket(hardwareMap)
        val spintake = Spintake(hardwareMap)


        telemetry.status("initialized servos")


        runBlocking {
            val auto = launch {
                val driveSpeed = 0.5
                val sideSpeed = 0.5
                val turnSpeed = 0.8

                spintake.pivotState(Spintake.SpintakePivotState.Dodge)

                parallelWait({
                    drivebase.driveOffsetGlobal(
                        xInches = -23.22,
                        yInches = 7.938,
                        angleRadians = 0.8,
                        maxPower = 0.5,
                        precise = false,
                    )
                }, {
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT)
                })

                lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT)
                flipper.pivotState(Bucket.BucketState.Out)
                delay(500) // give flipper time to extend
                flipper.pivotState(Bucket.BucketState.In)

                drivebase.driveForward(4.0, driveSpeed)

                parallelWait({
                    drivebase.driveOffsetGlobal(
                        xInches = -16.16,
                        yInches = 25.66,
                        angleRadians = 1.58,
                        maxPower = 0.5,
                        precise = true,
                    )

                    spintake.pivotState(Spintake.SpintakePivotState.Down)
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Spit)
                    delay(2000)
                }, {
                    lift.moveLiftTo(0.0)
                })

                parallelWait({
                    extender.extendTo(0.0, 0.5)
                }, {
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Off)
                    spintake.pivotState(Spintake.SpintakePivotState.Up)
                    delay(1000)
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Suck)
                    delay(1000)
                    spintake.pivotState(Spintake.SpintakePivotState.Dodge)
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Off)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT)
                }, {
                    drivebase.driveOffsetGlobal(
                        xInches = -23.22,
                        yInches = 7.938,
                        angleRadians = 0.8,
                        maxPower = 0.5,
                        precise = false,
                    )
                })

                lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT)

                flipper.pivotState(Bucket.BucketState.Out)
                delay(500)
                flipper.pivotState(Bucket.BucketState.In)

                drivebase.driveForward(4.0, driveSpeed)


                parallelWait({
                    drivebase.driveOffsetGlobal(
                        xInches = -25.71,
                        yInches = 25.66,
                        angleRadians = 1.58,
                        maxPower = 0.5,
                        precise = true,
                    )

                    spintake.pivotState(Spintake.SpintakePivotState.Down)
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Spit)
                    delay(2000)
                }, {
                    lift.moveLiftTo(0.0)
                })

                parallelWait({
                    extender.extendTo(0.0, 0.5)
                }, {
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Off)
                    spintake.pivotState(Spintake.SpintakePivotState.Up)
                    delay(1000)
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Suck)
                    delay(1000)
                    spintake.pivotState(Spintake.SpintakePivotState.Dodge)
                    spintake.controlIntakeState(Spintake.SpintakeIntakeState.Off)
                    lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT)
                }, {
                    drivebase.driveOffsetGlobal(
                        xInches = -23.22,
                        yInches = 7.938,
                        angleRadians = 0.8,
                        maxPower = 0.5,
                        precise = false,
                    )
                })

                lift.moveLiftTo(LiftConstants.MAX_LIFT_HEIGHT_LEFT)

                flipper.pivotState(Bucket.BucketState.Out)
                delay(500)
                flipper.pivotState(Bucket.BucketState.In)

                drivebase.driveForward(4.0, driveSpeed)

                parallelWait({
                    drivebase.driveOffsetGlobal(
                        xInches = 65.07,
                        yInches = 27.28,
                        angleRadians = 0.0,
                        maxPower = 0.5,
                        precise = false,
                    )
                    drivebase.driveOffsetGlobal(
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