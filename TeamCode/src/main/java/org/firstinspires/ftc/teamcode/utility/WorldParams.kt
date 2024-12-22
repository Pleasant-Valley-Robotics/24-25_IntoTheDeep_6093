package org.firstinspires.ftc.teamcode.utility

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
 * [this picture]()
 * for the deets
 */
fun paramsFromComponents(
): WorldParams {
    val flipperRotationDegrees = TODO()

    return flipperRotationDegrees
}
