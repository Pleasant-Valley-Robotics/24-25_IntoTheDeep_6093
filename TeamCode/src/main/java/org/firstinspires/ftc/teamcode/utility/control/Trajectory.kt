package org.firstinspires.ftc.teamcode.utility.control

interface Trajectory {
    fun getPoint(t: Double): PoseData
}
