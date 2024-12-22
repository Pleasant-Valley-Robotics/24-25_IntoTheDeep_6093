package org.firstinspires.ftc.teamcode.utility

import android.graphics.Canvas
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import kotlin.math.cos
import kotlin.math.roundToInt
import kotlin.math.sin

class ColorFilterPipeline : VisionProcessor {

    val params = FilterParams(
        aMin = 0.0,
        aMax = 255.0,
        bMin = 0.0,
        bMax = 255.0,
        aPerB = 0.0,
        bPerA = 0.0,
    )

//    val min = Scalar(0.0, 0.0, 0.0)
//    val max = Scalar(255.0, 255.0, 255.0)
//
//    val aPerB = 0f
//    val bPerA = 0f

    val decimationFactor = 16

    private lateinit var bufA: Mat
    private lateinit var bufB: Mat
    private lateinit var mask: Mat
    private lateinit var bigMask: Mat
    private val hierarchy = Mat()
    private val contours: MutableList<MatOfPoint> = mutableListOf()

    val maxContourPoints: List<Pair<Double, Double>>
        get() = maxContour.toList().map { it.x to it.y }

    private var maxContour = MatOfPoint()

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        bufA = Mat.zeros(height / decimationFactor, width / decimationFactor, CvType.CV_8UC3)
        bufB = Mat.zeros(height / decimationFactor, width / decimationFactor, CvType.CV_8UC3)
        mask = Mat.zeros(height / decimationFactor, width / decimationFactor, CvType.CV_8UC1)
        bigMask = Mat.zeros(height, width, CvType.CV_8UC1)
    }

    override fun processFrame(frame: Mat, processMs: Long): Pair<List<Point>?, Point?> {
        // not even gonna try to explain this, check the python code
        val minA = params.aMin.roundToInt()
        val minB = params.bMin.roundToInt()
        val maxA = params.aMax.roundToInt()
        val maxB = params.bMax.roundToInt()
        val aPerB = params.aPerB.toFloat()
        val bPerA = params.bPerA.toFloat()


        val shiftA = (minA + maxA) / -2
        val shiftB = (minB + maxB) / -2
        val det = 1 - aPerB * bPerA

        val detInv = 1 / det
        val detAB = -aPerB / det
        val detBA = -bPerA / det

        // easier to write out matmul by hand
        val boundShiftA = shiftA - detInv * shiftA - detAB * shiftB
        val boundShiftB = shiftB - detBA * shiftA - detInv * shiftB

        val minShiftA = minA + boundShiftA
        val minShiftB = minB + boundShiftB
        val maxShiftA = maxA + boundShiftA
        val maxShiftB = maxB + boundShiftB

        val data = floatArrayOf(
            1f, 0f, 0f,
            0f, detInv, detAB,
            0f, detBA, detInv,
        )

        val trans3dInv = Mat(3, 3, CvType.CV_32F)
        trans3dInv.put(0, 0, data)

        Imgproc.resize(frame, bufA, bufA.size(), Imgproc.INTER_NEAREST.toDouble())

        Imgproc.cvtColor(bufA, bufB, Imgproc.COLOR_RGB2Lab)
        Core.transform(bufB, bufA, trans3dInv)
        Core.inRange(
            bufA,
            Scalar(0.0, minShiftA.toDouble(), minShiftB.toDouble()),
            Scalar(255.0, maxShiftA.toDouble(), maxShiftB.toDouble()),
            mask
        )

        Imgproc.findContours(
            mask,
            contours,
            hierarchy,
            Imgproc.RETR_TREE,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        val maxContourOrNull = contours.maxByOrNull { Imgproc.contourArea(it) }
        val avgPoint = maxContourOrNull?.let { contour ->
            maxContour = contour
            contour.toList()
                .reduce { a, b -> Point(a.x + b.x, a.y + b.y) }
                .let { Point(it.x / contour.size(0), it.y / contour.size(0)) }
        }

        contours.clear()

        return Pair(maxContourOrNull?.toList(), avgPoint)
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        @Suppress("UNCHECKED_CAST")
        val listPointPair = userContext as Pair<List<Point>?, Point?>
        val points = listPointPair.first
        val point = listPointPair.second

        if (points == null || point == null) return

        val pointPaint = Paint()
        pointPaint.setARGB(255, 255, 255, 0)
        pointPaint.style = Paint.Style.FILL


        canvas.drawCircle(
            point.x.toFloat() * scaleBmpPxToCanvasPx * decimationFactor,
            point.y.toFloat() * scaleBmpPxToCanvasPx * decimationFactor,
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
                startPoint.x.toFloat() * scaleBmpPxToCanvasPx * decimationFactor,
                startPoint.y.toFloat() * scaleBmpPxToCanvasPx * decimationFactor,
                endPoint.x.toFloat() * scaleBmpPxToCanvasPx * decimationFactor,
                endPoint.y.toFloat() * scaleBmpPxToCanvasPx * decimationFactor,
                linePaint
            )
        }
    }

}
