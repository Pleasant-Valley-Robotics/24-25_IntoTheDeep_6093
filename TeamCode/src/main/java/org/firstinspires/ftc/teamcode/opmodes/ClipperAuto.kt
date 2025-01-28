package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.Clipper
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.LeftLift
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.RightLift
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_RIGHT
import kotlin.math.PI

@Autonomous(name = "ClipperAuto")
class ClipperAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val lift = RightLift(hardwareMap)
        val extender = Extender(hardwareMap)

        val bucket = Bucket(hardwareMap)
        val spintake = Spintake(hardwareMap)
        val pivot = Pivot(hardwareMap)
        val clipper = Clipper(hardwareMap)

        odometry.resetOdometry()
        lift.resetLift()
        extender.resetExtender()

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            suspend fun toPole(offset: Double) {
                drivebase.driveOffsetGlobal(-15.21 + offset, 15.79, -1.571, 0.5, false)
                drivebase.driveOffsetGlobal(-15.21 + offset, 26.39, -1.571, 0.5, true)
            }

            val auto = launch {
                pivot.movePivot(Pivot.PivotState.Dodge)
                clipper.moveClaw(Clipper.ClipperState.Closed)

                parallelWait(
                    { toPole(0.0) },
                    { lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 2.0) },
                )

                lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 7.0)
                clipper.moveClaw(Clipper.ClipperState.Open)
                parallelWait({
                    drivebase.driveOffsetGlobal(19.76, 25.04, 0.0, 1.0, false)
                    drivebase.strafeLeft(10.0, 1.0)
                    drivebase.driveForward(7.0, 1.0)
                    drivebase.strafeLeft(-10.0, 1.0)
                    drivebase.driveOffsetGlobal(46.03, 1.0, 0.0, 1.0, false)
                    drivebase.driveForward(-7.0, 1.0)
                    drivebase.driveOffsetGlobal(30.67, 12.49, 1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(31.95, 0.279, 1.571, 0.5, true)
                },
                    { lift.moveLiftTo(0.0) }
                )

                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(1000L)
                parallelWait(
                    { toPole(2.0) },
                    { lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 2.0) },
                )

                lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 7.0)
                clipper.moveClaw(Clipper.ClipperState.Open)

                parallelWait({
                    drivebase.driveOffsetGlobal(29.76 + 6.0, 25.04, 0.0, 1.0, false)
                    drivebase.strafeLeft(10.0, 1.0)
                    drivebase.driveForward(7.0, 1.0)
                    drivebase.strafeLeft(-10.0, 1.0)
                    drivebase.driveOffsetGlobal(46.03, 1.0, 0.0, 1.0, false)
                    drivebase.driveForward(-7.0, 1.0)
                    drivebase.driveOffsetGlobal(30.67, 12.49, 1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(31.95, 0.279, 1.571, 0.5, true)
                },
                    { lift.moveLiftTo(0.0) }
                )
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