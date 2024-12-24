package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.utility.vision.ColorFilterPipeline
import org.firstinspires.ftc.vision.VisionPortal

class Camera(hardwareMap: HardwareMap) {
    private val cameraName = hardwareMap.get(WebcamName::class.java, "Webcam 1")!!
    private val visionPortal = VisionPortal.easyCreateWithDefaults(
        cameraName,
        // ...processors =
        ColorFilterPipeline,
    )
}