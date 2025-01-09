package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utility.CameraConstants.TARGET_BLOCK_OFFSET_IN
import org.firstinspires.ftc.teamcode.utility.vision.BlockColor
import org.firstinspires.ftc.teamcode.utility.vision.SamplePipeline
import org.firstinspires.ftc.vision.VisionPortal

class Camera(hardwareMap: HardwareMap) {
    private val cameraName = hardwareMap.get(WebcamName::class.java, "Webcam 1")!!
    private val visionPortal = VisionPortal.easyCreateWithDefaults(
        cameraName,
        // ...processors =
        SamplePipeline,
    ).apply { setProcessorEnabled(SamplePipeline, false) }

    private fun Double.sqr() = this * this

    val nearestCenterError: Pair<Double, Double>
        get() = SamplePipeline
            .contourCenters
            .map { (x, y) -> x - TARGET_BLOCK_OFFSET_IN to y }
            .minBy { (x, y) -> x.sqr() + y.sqr() }

    var sampleColor: BlockColor? = null
        set(value) {
            if (value != null) {
                SamplePipeline.filterParams = value.getFilterParams()
            }
            field = value
        }

    var samplePipelineActive: Boolean
        get() = visionPortal.getProcessorEnabled(SamplePipeline)
        set(value) {
            visionPortal.setProcessorEnabled(SamplePipeline, value)
        }
}
