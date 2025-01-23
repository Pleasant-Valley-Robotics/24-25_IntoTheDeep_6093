package org.firstinspires.ftc.teamcode.utility.vision

import android.graphics.Canvas
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.utility.CameraConstants.BLOCK_HEIGHT_IN
import org.firstinspires.ftc.teamcode.utility.CameraConstants.SPINTAKE_HEIGHT_IN
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

/**
 * the general strategy for I/O on these pipelines is to have mutable public parameters,
 * and public value getters. these pipelines run on a separate thread, hopefully kotlin
 * respects the volatile field correctly.
 */
class SamplePipeline : VisionProcessor {
    @Volatile
    lateinit var filterParams: ColorFilter.FilterParams

    /**
     * it is recommended to use a pivot-local coordinate system,
     * with the camera a fixed distance from (x=0, y=0, z) and with rotation
     * (yaw=ZRot=0, roll=XRot=0). that way, proportional correction commands can be issued
     * directly using the calculated offsets from the target point.
     */
    @Volatile
    lateinit var cameraPose: PerspectiveTransform.CameraPose

    @Volatile
    lateinit var cameraParams: PerspectiveTransform.CameraParams
        private set

    private val allInitialized
        get() = ::filterParams.isInitialized
                && ::cameraPose.isInitialized
                && ::cameraParams.isInitialized

    @Volatile
    var contourCenters: List<Pair<Double, Double>> = emptyList()
        private set

    private fun Double.sqr() = this * this

    @Volatile
    private var maxContour: List<Pair<Double, Double>> = emptyList()

    @Volatile
    var maxContourCenter: Pair<Double, Double>? = null

    private val DECIMATION_FACTOR = 1

    // the buffers can be lateinit without checking. init runs before the buffers are read
    // in the processing code
    private lateinit var bufA: Mat
    private lateinit var bufB: Mat
    private lateinit var bufC: Mat
    private lateinit var mask: Mat
    private val hierarchy = Mat()
    private val contours: MutableList<MatOfPoint> = mutableListOf()

    private var newWidth: Int = 0
    private var newHeight: Int = 0

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        newWidth = width / DECIMATION_FACTOR
        newHeight = height / DECIMATION_FACTOR

        cameraParams = calibration.run {
            PerspectiveTransform.CameraParams(
                imWidth = newWidth,
                imHeight = newHeight,
                focalLengthX = focalLengthX.toDouble(),
                focalLengthY = focalLengthY.toDouble(),
                principalX = principalPointX.toDouble(),
                principalY = principalPointY.toDouble(),
                detectedZ = BLOCK_HEIGHT_IN - SPINTAKE_HEIGHT_IN,
            )
        }

        println(cameraParams)


        bufA = Mat.zeros(newHeight, newWidth, CvType.CV_8UC3)
        bufB = Mat.zeros(newHeight, newWidth, CvType.CV_8UC3)
        mask = Mat.zeros(newHeight, newWidth, CvType.CV_8UC1)
    }

    override fun processFrame(frame: Mat, processMs: Long): Any? {
        if (!allInitialized) return null

        Imgproc.resize(
            /* src = */ frame,
            /* dst = */ bufA,
            /* dsize = */ bufA.size(),
            /* fx = */ 0.0,
            /* fy = */ 0.0,
            /* interpolation = */ Imgproc.INTER_NEAREST,
        )

        Imgproc.cvtColor(bufA, bufB, Imgproc.COLOR_RGBA2RGB)
        Imgproc.cvtColor(bufB, bufB, Imgproc.COLOR_RGB2Lab)

        bufC = Mat.zeros(newHeight, newWidth, CvType.CV_8UC3)
        // modifies mask, bufB now has filtered image in it
        ColorFilter.colorFilter(bufB, filterParams, bufC)

        maxContourCenter = Imgproc.moments(bufC)
            .run { Pair(m10 / m00, m01 / m00) }
            .let { (x, y) -> Pair(x - cameraParams.principalX, y - cameraParams.principalY) }



        Imgproc.blur(bufC, bufB, Size(5.0, 5.0))

        Imgproc.Canny(
            /* image = */ bufB,
            /* edges = */ bufC,
            /* threshold1 = */ 25.0,
            /* threshold2 = */ 100.0,
            /* apertureSize = */ 3,
        )

        Imgproc.blur(bufC, bufB, Size(5.0, 5.0))

        Imgproc.findContours(
            /* image = */ bufB,
            /* contours = */ contours,
            /* hierarchy = */ hierarchy,
            /* mode = */ Imgproc.RETR_TREE,
            /* method = */ Imgproc.CHAIN_APPROX_SIMPLE,
        )

        contours.removeAll { Imgproc.contourArea(it) < 20 }

        contourCenters = contours
            .map { Imgproc.moments(it).run { Pair(m10 / m00, m01 / m00) } }
            .map {
                PerspectiveTransform.inversePerspective(
                    u = it.first,
                    v = it.second,
                    pose = cameraPose,
                    params = cameraParams,
                )
            }

//        maxContour = contourCenters
//            .map { (x, y) -> x - TARGET_BLOCK_OFFSET_IN to y }
//            .zip(contours)
//            .minByOrNull { (p, _) -> p.first.sqr() + p.second.sqr() }
//            ?.second
//            ?.toList()
//            ?.map { it.x to it.y }
//            ?: emptyList()

        maxContour = contours
            .maxByOrNull { Imgproc.contourArea(it) }
            ?.toList()
            ?.map { it.x to it.y }
            ?: emptyList()

//        maxContourCenter = maxContour
//            .reduceOrNull { (x1, y1), (x2, y2) -> Pair(x1 + x2, y1 + y2) }
//            ?.let { (x, y) -> Pair(x / maxContour.size, y / maxContour.size) }


        contours.clear()
        hierarchy.release()

        return frame
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        val pointPaint = Paint()
        pointPaint.setARGB(255, 255, 255, 0)
        pointPaint.style = Paint.Style.FILL

        val linePaint = Paint()
        linePaint.setARGB(255, 0, 255, 255)
        linePaint.style = Paint.Style.STROKE
        linePaint.strokeWidth = 5f

        val points = maxContour

        for ((px, py) in maxContourCenter?.let { listOf(it) } ?: emptyList()) {
            canvas.drawCircle(
                px.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                py.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                20f, pointPaint
            )
        }

        for (i in points.indices) {
            val (startX, startY) = points[i]
            val (endX, endY) = points[(i + 1) % points.size]
            canvas.drawLine(
                startX.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                startY.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                endX.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                endY.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                linePaint
            )
        }
    }

}
