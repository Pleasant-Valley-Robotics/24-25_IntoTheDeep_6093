package org.firstinspires.ftc.teamcode.utility.vision

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.core.Size

object ColorFilter {
    private val scratch = Mat()
    private val mask = Mat()
    private val trans3dInv = Mat(3, 3, CvType.CV_32F)

    private fun resize(size: Size) {
        scratch.create(size, CvType.CV_8UC3)
        mask.create(size, CvType.CV_8UC1)
    }

    /**
     * keeps pixels that are within [params] color limits.
     *
     * modifies [image] to the original pixel color if it matched the filter, black otherwise.
     *
     * @param image the image to filter, will be modified. datatype should be [CvType.CV_8UC3]
     * @param params the parameters for the color filter.
     */
    fun colorFilter(image: Mat, params: FilterParams, outMask: Mat) = params.run {
        if (scratch.size() != image.size()) resize(image.size())

        // not even gonna try to explain this, check the python code
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

        trans3dInv.put(
            0, 0, floatArrayOf(
                1f, 0f, 0f,
                0f, detInv, detAB,
                0f, detBA, detInv,
            )
        )

        Core.transform(image, scratch, trans3dInv)
        Core.inRange(
            /* src = */ scratch,
            /* lowerb = */ Scalar(0.0, minShiftA.toDouble(), minShiftB.toDouble()),
            /* upperb = */ Scalar(255.0, maxShiftA.toDouble(), maxShiftB.toDouble()),
            /* dst = */ mask
        )

//        image.copyTo(outMask, mask)
        mask.copyTo(outMask)
    }

    data class FilterParams(
        val minA: Int,
        val maxA: Int,
        val minB: Int,
        val maxB: Int,
        val aPerB: Float,
        val bPerA: Float,
    )
}
