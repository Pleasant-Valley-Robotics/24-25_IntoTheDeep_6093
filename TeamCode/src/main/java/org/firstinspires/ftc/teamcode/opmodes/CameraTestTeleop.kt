package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Camera
import org.firstinspires.ftc.teamcode.systems.Pivot
import org.firstinspires.ftc.teamcode.systems.Pivot.PivotState
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.CameraConstants.CAMERA_OFFSET_X_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.CAMERA_OFFSET_Y_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.CAMERA_RADIUS_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.SPINTAKE_DOWN_ANGLE_RAD
import org.firstinspires.ftc.teamcode.utility.vision.BlockColor
import org.firstinspires.ftc.teamcode.utility.vision.PerspectiveTransform
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "CameraTestTeleop")
class CameraTestTeleop : LinearOpMode() {
    override fun runOpMode() {
        val spintake = Spintake(hardwareMap)
        val pivot = Pivot(hardwareMap)
        val camera = Camera(hardwareMap)

        camera.sampleColor = BlockColor.Yellow
        camera.samplePipelineActive = true

        val pitch = SPINTAKE_DOWN_ANGLE_RAD
        val xOffset = CAMERA_OFFSET_X_IN
        val zOffset = CAMERA_RADIUS_IN

        val xNew = +xOffset * sin(pitch) - zOffset * cos(pitch)
        val zNew = +xOffset * cos(pitch) + zOffset * sin(pitch)

        camera.samplePipelineActive = true

        val pose = PerspectiveTransform.CameraPose(
            cameraX = xNew,
            cameraY = CAMERA_OFFSET_Y_IN,
            cameraZ = zNew,
            cameraXRot = PI / 2.0,
            cameraYRot = pitch,
            cameraZRot = 0.0,
        )

        camera.pose = pose

        pivot.movePivot(PivotState.Look)

        telemetry.status("initialized")
        waitForStart()

        runBlocking {
            val teleop = launch {

            }

            while (opModeIsActive()) {
                camera.addTelemetry(telemetry)
                telemetry.status("running")

                yield()
            }
        }

        telemetry.status("finished")
    }
}