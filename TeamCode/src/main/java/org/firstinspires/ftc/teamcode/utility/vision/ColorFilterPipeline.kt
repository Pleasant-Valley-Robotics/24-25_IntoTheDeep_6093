package org.firstinspires.ftc.teamcode.utility.vision

import android.graphics.Canvas
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.utility.CameraConstants.BLOCK_HEIGHT_IN
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

/**
 * the general strategy for I/O on these pipelines is to have mutable public parameters,
 * and public value getters. these pipelines run on a separate thread, hopefully kotlin
 * respects the volatile field correctly.
 */
object ColorFilterPipeline : VisionProcessor {
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

    lateinit var cameraParams: PerspectiveTransform.CameraParams
        private set

    private val allInitialized
        get() = ::filterParams.isInitialized
                && ::cameraPose.isInitialized
                && ::cameraParams.isInitialized

    val contourCenters: List<Pair<Double, Double>>
        get() {
            if (!allInitialized) return emptyList()

            return contours
                .map {
                    Imgproc.moments(it).run { Pair(cameraParams.imWidth - m10 / m00, m01 / m00) }
                }
                .map {
                    PerspectiveTransform.inversePerspective(
                        u = it.first,
                        v = it.second,
                        pose = cameraPose,
                        params = cameraParams,
                    )
                }
        }

    private const val DECIMATION_FACTOR = 16

    private lateinit var bufA: Mat
    private lateinit var bufB: Mat
    private lateinit var mask: Mat
    private lateinit var bigMask: Mat
    private val hierarchy = Mat()
    private val contours: MutableList<MatOfPoint> = mutableListOf()

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        cameraParams = calibration.run {
            PerspectiveTransform.CameraParams(
                imWidth = width / DECIMATION_FACTOR,
                imHeight = height / DECIMATION_FACTOR,
                focalLengthX = focalLengthX.toDouble(),
                focalLengthY = focalLengthY.toDouble(),
                principalX = principalPointX.toDouble(),
                principalY = principalPointY.toDouble(),
                detectedZ = BLOCK_HEIGHT_IN,
            )
        }


        bufA = Mat.zeros(height / DECIMATION_FACTOR, width / DECIMATION_FACTOR, CvType.CV_8UC3)
        bufB = Mat.zeros(height / DECIMATION_FACTOR, width / DECIMATION_FACTOR, CvType.CV_8UC3)
        mask = Mat.zeros(height / DECIMATION_FACTOR, width / DECIMATION_FACTOR, CvType.CV_8UC1)
        bigMask = Mat.zeros(height, width, CvType.CV_8UC1)
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

        Imgproc.cvtColor(bufA, bufB, Imgproc.COLOR_RGB2Lab)

        // modifies mask, bufB now has filtered image in it
        ColorFilter.colorFilter(bufB, filterParams)

        Imgproc.Canny(
            /* image = */ bufB,
            /* edges = */ bufA,
            /* threshold1 = */ 100.0,
            /* threshold2 = */ 200.0,
            /* apertureSize = */ 3,
        )

        Imgproc.findContours(
            /* image = */ bufA,
            /* contours = */ contours,
            /* hierarchy = */ hierarchy,
            /* mode = */ Imgproc.RETR_TREE,
            /* method = */ Imgproc.CHAIN_APPROX_SIMPLE,
        )

        contours.removeAll { Imgproc.contourArea(it) < 20 }

        val centers = contours.map { Imgproc.moments(it).run { Pair(m10 / m00, m01 / m00) } }

        return null
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        @Suppress("UNCHECKED_CAST", "SafeCastWithReturn")
        userContext as? Pair<List<Point>?, Point?> ?: return
        val points = userContext.first
        val point = userContext.second

        if (points == null || point == null) return

        val pointPaint = Paint()
        pointPaint.setARGB(255, 255, 255, 0)
        pointPaint.style = Paint.Style.FILL


        canvas.drawCircle(
            point.x.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
            point.y.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
            20f, pointPaint
        )


        val linePaint = Paint()
        linePaint.setARGB(255, 0, 255, 255)
        linePaint.style = Paint.Style.STROKE
        linePaint.strokeWidth = 5f

        for (i in points.indices) {
            val startPoint = points[i]
            val endPoint = points[(i + 1) % points.size]
            canvas.drawLine(
                startPoint.x.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                startPoint.y.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                endPoint.x.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                endPoint.y.toFloat() * scaleBmpPxToCanvasPx * DECIMATION_FACTOR,
                linePaint
            )
        }
    }

}
