package org.firstinspires.ftc.teamcode.utility.control

interface Trajectory {
//    val range: ClosedFloatingPointRange<Double>
    val start: Double
    val end: Double
    fun nearestPoint(pose: PoseData): Double
    fun pointsAround(pose: PoseData, dist: Double): List<Double>
    fun getPos(t: Double): PoseData
    fun getVel(t: Double): PoseData
}
