package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.systems.Clipper
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.RightLift
import kotlin.math.PI

@Autonomous(group = "Specimen", preselectTeleOp = "MainTeleop")
class ClipperAuto4V2 : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(
            hardwareMap, Pose2D(
                DistanceUnit.INCH, 40.636, -1.629,
                AngleUnit.RADIANS, -PI / 2,
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
            val auto = launch {
                clipper.moveClaw(Clipper.ClipperState.Closed)
                pivot.movePivot(Pivot.PivotState.Dodge)
                scoreSample(drivebase, lift, clipper, 0.5)

                parallelWait({
                    drivebase.driveOffsetGlobal(67.2637, 23.5902, -PI / 2, 1.0, false)
                    drivebase.driveOffsetGlobal(67.2637, 46.4816, -PI / 2, 1.0, false)
                    drivebase.driveOffsetGlobal(80.1976, 46.4816, -PI / 2, 1.0, false)
                    drivebase.driveOffsetGlobal(80.1976, 9.4049, -PI / 2, 1.0, false)
                    drivebase.driveOffsetGlobal(80.1976, 46.4816, -PI / 2, 1.0, false)

                    drivebase.driveOffsetGlobal(90.4633, 46.4816, -PI / 2, 1.0, false)
                    drivebase.driveOffsetGlobal(90.4633, 9.4049, -PI / 2 + 0.1, 1.0, false)
                }, { lift.moveLiftTo(0.0) })

                pickClip(drivebase, lift, clipper)
                scoreSample(drivebase, lift, clipper, 0.6)

                pickClip(drivebase, lift, clipper)
                scoreSample(drivebase, lift, clipper, 0.7)

                pickClip(drivebase, lift, clipper)
                scoreSample(drivebase, lift, clipper, 0.8)

                drivebase.driveOffsetGlobal(90.4633, 9.4049, -PI / 2, 1.0, false)
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