package org.firstinspires.ftc.teamcode.opmodes

import android.graphics.Path.Op
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.systems.Bucket
import org.firstinspires.ftc.teamcode.systems.Clipper
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.RightLift
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_RIGHT
import kotlin.math.PI

@Autonomous(name = "ClipperAuto4")
class ClipperAuto4 : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(
            hardwareMap, Pose2D(
                DistanceUnit.INCH, -4.2935, -0.2275,
                AngleUnit.RADIANS, -PI / 2
            )
        )

        LAST_AUTO_START_POS = odometry.poseOffset

        val drivebase = Drivebase(hardwareMap, odometry)
        val lift = RightLift(hardwareMap)
        val extender = Extender(hardwareMap)

        val pivot = Pivot(hardwareMap)
        val clipper = Clipper(hardwareMap)

        odometry.resetOdometry()
        lift.resetLift()
        extender.resetExtender()

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            suspend fun scorePole(offset: Double) {
                parallelWait({
                    drivebase.driveOffsetGlobal(-8.21 + offset, 15.79, -1.5, 1.0, false)
                    drivebase.driveOffsetGlobal(-8.21 + offset, 25.99, -1.571, 0.75, true)
                }, { lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 2.0) })

                lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 7.0)
                clipper.moveClaw(Clipper.ClipperState.Open)
            }

            val auto = launch {
                pivot.movePivot(Pivot.PivotState.Dodge)
                clipper.moveClaw(Clipper.ClipperState.Closed)
                scorePole(-4.0)

                parallelWait({
                    drivebase.driveOffsetGlobal(19.76, 25.04, -1.571, 1.0, false)

                    drivebase.driveOffsetGlobal(19.76, 48.04, -1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(29.76, 48.04, -1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(29.76, 10.04, -1.571, 1.0, false)

                    drivebase.driveOffsetGlobal(29.76, 48.04, -1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(39.76, 48.04, -1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(39.76, 10.04, -1.571, 1.0, false)

                    drivebase.driveOffsetGlobal(21.95, 5.879, 1.3, 1.0, false)
                    drivebase.driveOffsetGlobal(21.95, 1.014, 1.571, 1.0, false)
                }, { lift.moveLiftTo(0.0) })

                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(250L)
                scorePole(-1.0)

                parallelWait({
                    drivebase.driveOffsetGlobal(21.95, 5.879, 1.5, 1.0, false)
                    drivebase.driveOffsetGlobal(21.95, 1.014, 1.571, 1.0, false)
                }, { lift.moveLiftTo(0.0) })


                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(250L)
                scorePole(2.0)

                parallelWait({
                    drivebase.driveOffsetGlobal(21.95, 5.879, 1.5, 1.0, false)
                    drivebase.driveOffsetGlobal(21.95, 1.014, 1.571, 1.0, false)
                }, { lift.moveLiftTo(0.0) })


                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(250L)

                parallelWait({
                    drivebase.driveOffsetGlobal(-8.21 + 5.0, 15.79, -1.5, 1.0, false)
                    drivebase.driveOffsetGlobal(-8.21 + 5.0, 26.39, -1.571, 0.5, true)
                }, { lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 2.25) })

                clipper.moveClaw(Clipper.ClipperState.Open)
                delay(100L)
                drivebase.driveOffsetGlobal(21.95, 1.014, -1.571, 1.0, false)
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

            clipper.moveClaw(Clipper.ClipperState.Open)
            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.setLiftPowerSafe(0.0, true)
        }
    }
}