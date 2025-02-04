package org.firstinspires.ftc.teamcode.opmodes.autos

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.opmodes.status
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.utility.control.LinearSpline
import org.firstinspires.ftc.teamcode.utility.control.PathController
import org.firstinspires.ftc.teamcode.utility.control.PoseData
import org.firstinspires.ftc.teamcode.utility.control.Vec2d

@Autonomous
class LinearSplineTestAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing")

        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val spline = LinearSpline(
            speed = 0.5,
            PoseData(Vec2d(0.0, 0.0), 0.0),
            PoseData(Vec2d(10.0, 10.0), 0.0),
            PoseData(Vec2d(10.0, 0.0), 0.0),
            PoseData(Vec2d(0.0, 0.0), 0.0),
        )
        val controller = PathController(drivebase, odometry)

        odometry.resetOdometry()
        drivebase.resetMotorEncoders()

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            val auto = launch {
                controller.driveTrajectory(spline, 2.0, 0.5)
            }

            while (opModeIsActive() && auto.isActive) {
                odometry.addTelemetry(telemetry)
                telemetry.status("running")

                odometry.update()
                yield()
            }

            drivebase.controlMotors(0.0, 0.0, 0.0)
        }
    }
}