package org.firstinspires.ftc.teamcode.utility.vision

import kotlin.math.cos
import kotlin.math.sin

object PerspectiveTransform {
    /**
     * goes from screen space coordinates (u, v) to global coordinates (x, y)
     *
     * @param u image x coordinate
     * @param v image y coordinate
     * @param pose the global position of the camera
     * @param params the intrinsic parameters of the camera
     * @return a pair (x, y) in global coordinates
     */
    fun inversePerspective(
        u: Double,
        v: Double,
        pose: CameraPose,
        params: CameraParams
    ): Pair<Double, Double> {
        // all of this is just matrix multiplications groupCount() expanded manually for efficiency and
        // because i hate working with the opencv matrix library.

        // note heavy use of scoped functions to keep good variable names from making
        // the code take a ton of screen space

        val rotation = let {
            val rx = -Math.toRadians(pose.cameraXRot)
            val ry = -Math.toRadians(pose.cameraYRot)
            val rz = -Math.toRadians(pose.cameraZRot)

            val sx = sin(rx)
            val sy = sin(ry)
            val sz = sin(rz)
            val cx = cos(rx)
            val cy = cos(ry)
            val cz = cos(rz)

            listOf(
                cy * cz, -sz * cy, sy,
                sx * sy * cz + sz * cx, -sx * sy * sz + cx * cz, -sx * cy,
                sx * sz - sy * cx * cz, sx * cz + sy * sz * cx, cx * cy,
            )
        }

        val translation = rotation.let { r ->
            val px = pose.cameraX
            val py = pose.cameraY
            val pz = pose.cameraZ

            listOf(
                -(r[0] * px + r[1] * py + r[2] * pz),
                -(r[3] * px + r[4] * py + r[5] * pz),
                -(r[6] * px + r[7] * py + r[8] * pz),
            )
        }

        val fullMatrix = (rotation to translation).let { (r, t) ->
            val fx = params.focalLengthX
            val fy = params.focalLengthY
            val cx = params.principalX
            val cy = params.principalY
            val zd = params.detectedZ

            listOf(
                fx * r[0] + cx * r[6],
                fx * r[1] + cx * r[7],
                fx * t[0] + cx * t[2] + zd * (fx * r[2] + cx * r[8]),

                fy * r[3] + cy * r[6],
                fy * r[4] + cy * r[7],
                fy * t[1] + cy * t[2] + zd * (fy * r[5] + cy * r[8]),

                r[6],
                r[7],
                r[8] * zd + t[2],
            )
        }

        // we do not actually need the inverse of the matrix, just the adjugate.
        // this is because the factor of 1/det gets cancelled by the conversion
        // from homogenous coordinates.
        val fullAdjugate = fullMatrix.let { f ->
            listOf(
                f[4] * f[8] - f[5] * f[7],
                f[2] * f[7] - f[1] * f[8],
                f[1] * f[5] - f[2] * f[4],
                f[5] * f[6] - f[3] * f[8],
                f[0] * f[8] - f[2] * f[6],
                f[2] * f[3] - f[0] * f[5],
                f[3] * f[7] - f[4] * f[6],
                f[1] * f[6] - f[0] * f[7],
                f[0] * f[4] - f[1] * f[3],
            )
        }

        val worldCoords = fullAdjugate.let { f ->
            // in the image 0, 0 is the top left. in the transform 0, 0 is the top right.
            // correct for flipped coordinates
            val lu = params.imWidth - u

            listOf(
                lu * f[0] + v * f[1] + f[2],
                lu * f[3] + v * f[4] + f[5],
                lu * f[6] + v * f[7] + f[8],
            )
        }

        val (wx, wy, w) = worldCoords

        val worldX = wx / w
        val worldY = wy / w

        return Pair(worldX, worldY)
    }

    data class CameraParams(
        val imWidth: Int,
        val imHeight: Int,
        val focalLengthX: Double,
        val focalLengthY: Double,
        val principalX: Double,
        val principalY: Double,
        val detectedZ: Double,
    )

    /**
     * @param cameraX units in cm
     * @param cameraY units in cm
     * @param cameraZ units in cm
     * @param cameraXRot units in deg
     * @param cameraYRot units in deg
     * @param cameraZRot units in deg
     */
    data class CameraPose(
        val cameraX: Double,
        val cameraY: Double,
        val cameraZ: Double,
        val cameraXRot: Double,
        val cameraYRot: Double,
        val cameraZRot: Double,
    )

    /**
     * check out
     * [this picture](https://github.com/Pleasant-Valley-Robotics/24-25_IntoTheDeep_6093/blob/207c26a0de0c07db1fce17a89ff0c34c374af4c8/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/documentation/camera_setup.png)
     * for the deets
     *
     * all length units in inches, all angle units in degrees
     *
     * @param cameraRadius inches
     * @param cameraOffset inches
     * @param extensionDistance inches
     * @param pivotAngle degrees
     *
     * @return pose for the camera
     */
    fun poseFromComponents(
        cameraRadius: Double,
        cameraOffset: Double,
        extensionDistance: Double,
        pivotAngle: Double,
    ): CameraPose {
        val s = sin(Math.toRadians(pivotAngle))
        val c = cos(Math.toRadians(pivotAngle))

        val localX = -cameraRadius * c - cameraOffset * s
        val localY = cameraRadius * s + cameraOffset * c

        val cameraXcm = (localX + extensionDistance) * 2.54
        val cameraZcm = localY * 2.54

        return CameraPose(
            cameraX = cameraXcm,
            cameraY = 0.0,
            cameraZ = cameraZcm,
            cameraXRot = 0.0,
            cameraYRot = pivotAngle,
            cameraZRot = 0.0,
        )
    }
}