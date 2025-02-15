package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.opmodes.moveToBasket
import org.firstinspires.ftc.teamcode.opmodes.parallelRace
import org.firstinspires.ftc.teamcode.opmodes.parallelWait
import org.firstinspires.ftc.teamcode.opmodes.status
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
            fun getGrabBlock(blockX: Double, blockY: Double, blockAng: Double) = suspend {
                drivebase.driveToPositionGlobal(
                    blockX,
                    blockY,
                    blockAng,
                    1.0,
                    false
                )
            }

            val blocks = listOf(
                getGrabBlock(-20.98, 14.97, 1.3628),
                getGrabBlock(-24.87, 13.97, 1.6714),
                getGrabBlock(-24.46, 17.07, 2.1798),
            )

            suspend fun score() {
                parallelWait(
                    { moveToBasket(drivebase) },
                    { lift.moveLiftTo(MAX_LIFT_HEIGHT_LEFT - 2.0) },
                )

                lift.moveLiftTo(MAX_LIFT_HEIGHT_LEFT - 2.0)

                bucket.moveBucket(Bucket.BucketState.Out)
                delay(500)
                bucket.moveBucket(Bucket.BucketState.In)
            }

            suspend fun scoreThenGrabUsing(
                beforeGrab: suspend () -> Unit,
                afterGrab: suspend () -> Unit,
            ) {
                score()

                parallelWait({
                    parallelWait({
                        drivebase.driveForward(4.0, 1.0)
                        beforeGrab()
                    }, {
                        extender.extendTo(MAX_EXTENSION - 1.0, 1.0)
                    })

                    spintake.controlIntakeState(SpintakeState.Suck)
                    pivot.movePivot(PivotState.Down)
                    sleep(1000)

                    spintake.controlIntakeState(SpintakeState.Off)
                    pivot.movePivot(PivotState.Up)
                    extender.extendTo(0.0, 1.0)
                }, {
                    lift.moveLiftTo(0.0)
                })

                parallelRace({
                    spintake.controlIntakeState(SpintakeState.Spit)
                    sleep(500)

                    spintake.controlIntakeState(SpintakeState.Off)
                    pivot.movePivot(PivotState.Dodge)
                }, {
                    afterGrab()
                    moveToBasket(drivebase, 1.0)
                })

                afterGrab()
            }

            val auto = launch {
                pivot.movePivot(PivotState.Dodge)

                for (moveToBlock in blocks) {
                    scoreThenGrabUsing({ moveToBlock() }, {})
                }

                scoreThenGrabUsing({
                    drivebase.driveToPositionGlobal(
                        58.98,
                        2.148,
                        0.0,
                        1.0,
                        true,
                    )
                }, {
                    parallelRace({
                        drivebase.driveToPositionGlobal(
                            xInches = -18.22,
                            yInches = 2.148,
                            angleRadians = 0.0,
                            maxPower = 1.0,
                            precise = false,
                        )
                    }, { lift.moveLiftTo(MAX_LIFT_HEIGHT_LEFT - 2.0) })
                })

                score()
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