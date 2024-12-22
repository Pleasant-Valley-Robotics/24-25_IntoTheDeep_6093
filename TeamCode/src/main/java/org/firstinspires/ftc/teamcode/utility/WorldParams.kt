package org.firstinspires.ftc.teamcode.utility

import kotlin.math.cos
import kotlin.math.sin

/**
 * @param targetX units in cm
 * @param targetY units in cm
 * @param cameraX units in cm
 * @param cameraY units in cm
 * @param cameraZ units in cm
 * @param cameraXRot units in deg
 * @param cameraYRot units in deg
 * @param cameraZRot units in deg
 * @param imWidth units in px
 * @param imHeight units in px
 * @param sensorWidth units in mm
 * @param focalLength units in mm
 * @param detectedZ units in cm
 */
data class WorldParams(
    val targetX: Double,
    val targetY: Double,
    val cameraX: Double,
    val cameraY: Double,
    val cameraZ: Double,
    val cameraXRot: Double,
    val cameraYRot: Double,
    val cameraZRot: Double,
    val imWidth: Double,
    val imHeight: Double,
    val sensorWidth: Double,
    val focalLength: Double,
    val detectedZ: Double,
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
fun paramsFromComponents(
    cameraRadius: Double,
    cameraOffset: Double,
    extensionDistance: Double,
    pivotAngle: Double,
): WorldParams {
    val s = sin(Math.toRadians(pivotAngle))
    val c = cos(Math.toRadians(pivotAngle))

    val lx = -cameraRadius * c - cameraOffset * s
    val ly = cameraRadius * s + cameraOffset * c

    val cx = lx + extensionDistance
    val cy = ly

    val params = WorldParams(
        targetX = TODO(),
        targetY = TODO(),
        cameraX = TODO(),
        cameraY = TODO(),
        cameraZ = TODO(),
        cameraXRot = TODO(),
        cameraYRot = TODO(),
        cameraZRot = TODO(),
        imWidth = TODO(),
        imHeight = TODO(),
        sensorWidth = TODO(),
        focalLength = TODO(),
        detectedZ = TODO()
    )
    return TODO()
}
