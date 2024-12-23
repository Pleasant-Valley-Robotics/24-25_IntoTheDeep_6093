package org.firstinspires.ftc.teamcode.utility.vision

import kotlin.math.cos
import kotlin.math.sin

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
