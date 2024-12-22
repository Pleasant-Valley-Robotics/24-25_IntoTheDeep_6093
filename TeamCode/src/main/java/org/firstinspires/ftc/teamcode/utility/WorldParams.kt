package org.firstinspires.ftc.teamcode.utility

/**
 * @param pose world position of camera
 * @param targetX units in cm
 * @param targetY units in cm
 * @param imWidth units in px
 * @param imHeight units in px
 * @param sensorWidth units in mm
 * @param focalLength units in mm
 * @param detectedZ units in cm
 */
data class WorldParams(
    val pose: CameraPose,
    val targetX: Double,
    val targetY: Double,
    val imWidth: Double,
    val imHeight: Double,
    val sensorWidth: Double,
    val focalLength: Double,
    val detectedZ: Double,
)
