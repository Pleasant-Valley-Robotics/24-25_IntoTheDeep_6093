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
import kotlin.math.pow
import kotlin.math.sin

/**
 * the general strategy for I/O on these pipelines is to have mutable public parameters,
 * and public value getters. these pipelines run on a separate thread, hopefully kotlin
 * respects the volatile field correctly.
 */
object ColorFilterPipeline : VisionProcessor {
    @Volatile
    var filterParams = FilterParams(
        aMin = 0,
        aMax = 255,
        bMin = 0,
        bMax = 255,
        aPerB = 0.0,
        bPerA = 0.0,
    )

    /**
     * the name *world*Params is a bit of a misnomer. the pipeline can indeed handle global
     * coordinates, but it is likely more useful to use a pivot-local coordinate system,
     * with the camera a fixed distance from (x=0, y=0, z) and with rotation
     * (yaw=ZRot=0, roll=XRot=0). that way, proportional correction commands can be issued
     * directly using the calculated offsets from the target point.
     */
    @Volatile
    var worldParams = WorldParams(
        targetX = 0.0,
        targetY = 0.0,
        pose = poseFromComponents(
            cameraRadius = 0.0,
            cameraOffset = 0.0,
            extensionDistance = 0.0,
            pivotAngle = 0.0
        ),
        imWidth = 0,
        imHeight = 0,
        sensorWidth = 0.0,
        focalLength = 0.0,
        detectedZ = 0.0,
    )

    val targetPointOffset: Pair<Double, Double>?
        get() = closestPoint?.let { (x, y) -> x - worldParams.targetX to y - worldParams.targetY }

    private var closestPoint: Pair<Double, Double>? = null

    private val decimationFactor = 16

    private lateinit var bufA: Mat
    private lateinit var bufB: Mat
    private lateinit var mask: Mat
    private lateinit var bigMask: Mat
    private val hierarchy = Mat()
    private val contours: MutableList<MatOfPoint> = mutableListOf()

    override fun init(width: Int, height: Int, calibration: CameraCalibration) {
        bufA = Mat.zeros(height / decimationFactor, width / decimationFactor, CvType.CV_8UC3)
        bufB = Mat.zeros(height / decimationFactor, width / decimationFactor, CvType.CV_8UC3)
        mask = Mat.zeros(height / decimationFactor, width / decimationFactor, CvType.CV_8UC1)
        bigMask = Mat.zeros(height, width, CvType.CV_8UC1)
    }

    /**
     * goes from screen space coordinates (u, v) to global coordinates (x, y)
     *
     * @param u image x coordinate
     * @param v image y coordinate
     * @param params parameters to use for transformation
     * @return a pair (x, y) in global coordinates
     */

    fun inversePerspective(u: Double, v: Double, params: WorldParams): Pair<Double, Double> {
        val f = params.focalLength * params.imWidth / params.sensorWidth

        val ix = params.imWidth / 2
        val iy = params.imHeight / 2

        val rx = -Math.toRadians(params.pose.cameraXRot)
        val ry = -Math.toRadians(params.pose.cameraYRot)
        val rz = -Math.toRadians(params.pose.cameraZRot)

        val sx = sin(rx)
        val sy = sin(ry)
        val sz = sin(rz)
        val cx = cos(rx)
        val cy = cos(ry)
        val cz = cos(rz)

        val zd = params.detectedZ

        val px = params.pose.cameraX
        val py = params.pose.cameraY
        val pz = params.pose.cameraZ

        // all of this is just matrix multiplications expanded manually for efficiency and
        // because i hate working with the opencv matrix library.

        val rot = listOf(
            cy * cz, -sz * cy, sy,
            sx * sy * cz + sz * cx, -sx * sy * sz + cx * cz, -sx * cy,
            sx * sz - sy * cx * cz, sx * cz + sy * sz * cx, cx * cy,
        )

        val trans = listOf(
            -(rot[0] * px + rot[1] * py + rot[2] * pz),
            -(rot[3] * px + rot[4] * py + rot[5] * pz),
            -(rot[6] * px + rot[7] * py + rot[8] * pz),
        )

        val fm = listOf(
            rot[0] * f + rot[6] * ix,
            rot[1] * f + rot[7] * ix,
            trans[2] * ix + trans[0] * f + zd * (rot[2] * f + rot[8] * ix),
            rot[6] * iy + rot[3] * f,
            rot[7] * iy + rot[4] * f,
            trans[2] * iy + trans[1] * f + zd * (rot[8] * iy + rot[5] * f),
            rot[6], rot[7], rot[8] * zd + trans[2],
        )

        // we do not actually need the inverse of the matrix, just the adjugate.
        // this is because the factor of 1/det gets cancelled by the conversion
        // from homogenous coordinates.
        val fa = listOf(
            fm[4] * fm[8] - fm[5] * fm[7],
            fm[2] * fm[7] - fm[1] * fm[8],
            fm[1] * fm[5] - fm[2] * fm[4],
            fm[5] * fm[6] - fm[3] * fm[8],
            fm[0] * fm[8] - fm[2] * fm[6],
            fm[2] * fm[3] - fm[0] * fm[5],
            fm[3] * fm[7] - fm[4] * fm[6],
            fm[1] * fm[6] - fm[0] * fm[7],
            fm[0] * fm[4] - fm[1] * fm[3],
        )

        // in the image 0, 0 is the top left. in the transform 0, 0 is the top right.
        // correct for flipped coordinates
        val lu = params.imWidth - u
        val worldCoords = listOf(
            lu * fa[0] + v * fa[1] + fa[2],
            lu * fa[3] + v * fa[4] + fa[5],
            lu * fa[6] + v * fa[7] + fa[8],
        )

        val wx = worldCoords[0]
        val wy = worldCoords[1]
        val w = worldCoords[2]

        val worldX = wx / w
        val worldY = wy / w

        return Pair(worldX, worldY)
    }

    /**
     * filters an image based on some params.
     * modifies [image], [bufA], and [mask].
     * only filters the last 2 channels of [image].
     *
     * @param image the image to filter, must be 8 bit 3 channel color.
     */
    fun colorFilter(image: Mat, params: FilterParams) {
        // not even gonna try to explain this, check the python code
        val minA = params.aMin
        val minB = params.bMin
        val maxA = params.aMax
        val maxB = params.bMax
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

        Core.transform(image, bufA, trans3dInv)
        Core.inRange(
            /* src = */ bufA,
            /* lowerb = */ Scalar(0.0, minShiftA.toDouble(), minShiftB.toDouble()),
            /* upperb = */ Scalar(255.0, maxShiftA.toDouble(), maxShiftB.toDouble()),
            /* dst = */ mask
        )
        Core.bitwise_and(bufA, bufA, image, mask)
    }

    override fun processFrame(frame: Mat, processMs: Long): Pair<List<Point>?, Point?> {
        val (_, _) = inversePerspective(1.0, 1.0, worldParams)

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
        colorFilter(bufB, filterParams)

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

        val centers = contours
            .map { Imgproc.moments(it) }
            .map { Pair(it.m10 / it.m00, it.m01 / it.m00) }

        val worldCenters = centers.map {
            inversePerspective(
                u = it.first,
                v = it.second,
                params = worldParams,
            )
        }

        val closestPointIndex = worldCenters.withIndex().minByOrNull { (_, p) ->
            (worldParams.targetX - p.first).pow(2) + (worldParams.targetY - p.second).pow(2)
        }

        if (closestPointIndex == null) return Pair(null, null)

        val (index, point) = closestPointIndex
        closestPoint = point

        return Pair(contours[index].toList(), Point(point.first, point.second))
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
