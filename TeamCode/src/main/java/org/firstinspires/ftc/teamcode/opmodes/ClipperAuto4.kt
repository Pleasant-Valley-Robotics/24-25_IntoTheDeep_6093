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
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.RightLift
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_RIGHT

@Autonomous(name = "ClipperAuto4")
class ClipperAuto4 : LinearOpMode() {
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
            suspend fun scorePole(offset: Double) {
                parallelWait(
                    {
                        drivebase.driveOffsetGlobal(-15.21 + offset, 15.79, -1.571, 0.5, false)
                        drivebase.driveOffsetGlobal(-15.21 + offset, 26.39, -1.571, 0.5, true)
                    },
                    { lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 2.0) },
                )

                lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 7.0)
                clipper.moveClaw(Clipper.ClipperState.Open)
            }

            val auto = launch {
                pivot.movePivot(Pivot.PivotState.Dodge)
                clipper.moveClaw(Clipper.ClipperState.Closed)
                scorePole(0.0)

                parallelWait({
                    drivebase.driveOffsetGlobal(19.76, 25.04, 0.0, 1.0, false)

                    drivebase.driveOffsetGlobal(19.76, 48.04, 0.0, 1.0, false)
                    drivebase.driveOffsetGlobal(29.76, 48.04, 0.0, 1.0, false)
                    drivebase.driveOffsetGlobal(29.76, 10.04, 0.0, 1.0, false)

                    drivebase.driveOffsetGlobal(29.76, 48.04, 0.0, 1.0, false)
                    drivebase.driveOffsetGlobal(39.76, 48.04, 0.0, 1.0, false)
                    drivebase.driveOffsetGlobal(39.76, 10.04, 0.0, 1.0, false)

                    drivebase.driveOffsetGlobal(31.95, 5.879, 1.571, 1.0, false)
                    drivebase.driveOffsetGlobal(31.95, 1.014, 1.571, 0.5, true)
                },
                    { lift.moveLiftTo(0.0) }
                )

                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(250L)
                scorePole(3.0)

                parallelWait({
                    drivebase.driveOffsetGlobal(31.95, 5.879, 1.571, 0.5, false)
                    drivebase.driveOffsetGlobal(31.95, 1.014, 1.571, 0.5, true)
                },
                    { lift.moveLiftTo(0.0) }
                )


                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(250L)
                scorePole(6.0)

                parallelWait({
                    drivebase.driveOffsetGlobal(31.95, 5.879, 1.571, 0.5, false)
                    drivebase.driveOffsetGlobal(31.95, 1.014, 1.571, 0.5, true)
                },
                    { lift.moveLiftTo(0.0) }
                )


                clipper.moveClaw(Clipper.ClipperState.Closed)
                delay(250L)
                scorePole(6.0)
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