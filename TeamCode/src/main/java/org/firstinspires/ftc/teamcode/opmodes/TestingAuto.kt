package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.utility.vision.BlockColor
import kotlin.math.PI

@Autonomous(name = "TestingAuto")
class TestingAuto : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing Drivebase")
        val odometry = Odometry(hardwareMap)
        val drivebase = Drivebase(hardwareMap, odometry)
        val camera = Camera(hardwareMap)

        camera.sampleColor = BlockColor.Yellow
        camera.samplePipelineActive = true

        telemetry.status("Initialized")
        waitForStart()

        runBlocking {
            val auto = launch {
                drivebase.driveOffsetGlobal(
                    xInches = 10.0,
                    yInches = 0.0,
                    angleRadians = 0.0,
                    maxPower = 1.0
                )
            }

            while (opModeIsActive() && auto.isActive) {
                camera.addTelemetry(telemetry)

                drivebase.addTelemetry(telemetry)
                odometry.addTelemetry(telemetry)

                telemetry.status("Running")

                odometry.update()

                yield()
            }

            // stop motors
            drivebase.controlMotors(0.0, 0.0, 0.0)

            auto.cancelAndJoin()
        }
    }
}