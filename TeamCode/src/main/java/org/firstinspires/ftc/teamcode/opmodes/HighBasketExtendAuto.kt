package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.Pivot.PivotState
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.systems.Spintake.SpintakeState
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants.MAX_EXTENSION
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_LEFT

@Autonomous(group = "Basket", preselectTeleOp = "MainTeleop")
class HighBasketExtendAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(hardwareMap)
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
            suspend fun grabBlock() {
                spintake.controlIntakeState(SpintakeState.Suck)
                pivot.movePivot(PivotState.Down)
                delay(1250)
                spintake.controlIntakeState(SpintakeState.Off)
                pivot.movePivot(PivotState.Dodge)
            }

            suspend fun dropBlock() {
                spintake.controlIntakeState(SpintakeState.Spit)
                pivot.movePivot(PivotState.Up)
                delay(750)
                pivot.movePivot(PivotState.Dodge)
                spintake.controlIntakeState(SpintakeState.Off)
            }

            suspend fun scoreBlock() {
                parallelWait(
                    { moveToBasket(drivebase) },
                    { lift.moveLiftTo(MAX_LIFT_HEIGHT_LEFT) },
                    { extender.extendTo(MAX_EXTENSION - 1.0, 1.0) },
                )

//                lift.moveLiftTo(MAX_LIFT_HEIGHT_LEFT)

                bucket.moveBucket(Bucket.BucketState.Out)
                delay(750)
                bucket.moveBucket(Bucket.BucketState.In)

//                drivebase.driveForward(4.0, 1.0)
            }

            suspend fun goGrabBlock(blockX: Double, blockY: Double, angle: Double) {
                parallelWait(
                    {
                        drivebase.driveOffsetGlobal(
                            xInches = blockX,
                            yInches = blockY,
                            angleRadians = angle,
                            maxPower = 0.5,
                            precise = true,
                        )
                        extender.extendTo(MAX_EXTENSION - 1.8, 1.0)
                        grabBlock()
                    },
                    { lift.moveLiftTo(1.4) },
                )

                extender.extendTo(0.0, 1.0)
//                lift.moveLiftTo(1.4)
                dropBlock()
            }

            val auto = launch {
                scoreBlock()
                goGrabBlock(-20.98, 14.97, 1.3628)
                scoreBlock()
                goGrabBlock(-24.87, 13.97, 1.6714)
                scoreBlock()
                goGrabBlock(-24.46, 17.07, 2.1198)
                scoreBlock()
            }

            while (opModeIsActive() && auto.isActive) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)
                odometry.addTelemetry(telemetry)
                telemetry.status("running")

                odometry.update()
                yield()
            }

            auto.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.setLiftPowerSafe(0.0, true)
        }
    }
}