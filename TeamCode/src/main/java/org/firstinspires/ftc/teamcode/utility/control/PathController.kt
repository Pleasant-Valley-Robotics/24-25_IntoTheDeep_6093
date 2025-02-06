package org.firstinspires.ftc.teamcode.utility.control

import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Odometry
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.DRIVING_D_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.DRIVING_I_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.DRIVING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.MOVEMENT_TOL_INCH_TIGHT
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.STRAFING_D_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.STRAFING_I_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.STRAFING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_D_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_I_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_P_GAIN
import org.firstinspires.ftc.teamcode.utility.DrivebaseConstants.TURNING_TOL_DEG_TIGHT
import org.firstinspires.ftc.teamcode.utility.TAU
import kotlin.math.absoluteValue
import kotlin.math.floor

class PathController(
    val drivebase: Drivebase,
    val odometry: Odometry,
) {
    val xPID = PidController(
        DRIVING_P_GAIN,
        DRIVING_I_GAIN,
        DRIVING_D_GAIN,
        0.2,
        0.0,
    ) { -odometry.localVelX }
    val yPID = PidController(
        STRAFING_P_GAIN,
        STRAFING_I_GAIN,
        STRAFING_D_GAIN,
        0.2,
        0.0,
    ) { -odometry.localVelY }
    val angPID = PidController(
        TURNING_P_GAIN,
        TURNING_I_GAIN,
        TURNING_D_GAIN,
        0.2,
        0.0,
    ) { -odometry.velRad }
//    val xPID = ClampController(0.5, 0.0)
//    val yPID = ClampController(0.5, 0.0)
//    val angPID = ClampController(1.0, 0.0)

    suspend fun driveTrajectory(
        trajectory: Trajectory,
        lookahead: Double,
        maxPower: Double
    ) {
        xPID.maxValue = maxPower
        yPID.maxValue = maxPower
        angPID.maxValue = maxPower

        val endPoint = trajectory.end
        var setPoint = trajectory.start

        do {
            // find closest (path-wise) point one lookahead distance along the track
            val currentPose = odometry.globalPose
            val nearestPoints = trajectory.nearestPoints(currentPose)
            val rangePoints = trajectory.pointsAround(currentPose, lookahead)
            val look = rangePoints
                .filter { it in setPoint..(setPoint + 1.0) }
                .minByOrNull { it - setPoint }

            setPoint = look ?: setPoint
            val (posError, angError) = trajectory.getPos(setPoint) - currentPose
            val (xError, yError) = posError.rotate(-currentPose.angRad)

            val (endPosError, endAngError) = trajectory.getPos(endPoint) - currentPose

            val xInput = xPID.accept(xError)
            val yInput = yPID.accept(yError)
            val angInput = angPID.accept(angError)

            val angVel: Double
            val xVel: Double
            val yVel: Double

            val atEnd = floor(endPoint - 0.01) == floor(setPoint - 0.01)

            if (atEnd || look == null) {
                angVel = 0.0
                xVel = 0.0
                yVel = 0.0
            } else {
                val thing = trajectory.getVel(setPoint)
                val rot = thing.pos.rotate(-currentPose.angRad)
                angVel = thing.angRad
                xVel = rot.x
                yVel = rot.y
            }

            drivebase.controlMotors(xInput + xVel, yInput + yVel, angInput + angVel)

            yield()
        } while (
//            nearest == null
            !atEnd
//            || (trajectory.getPos(endPoint) - trajectory.getPos(setPoint)).pos.length < lookahead
            || endPosError.length > MOVEMENT_TOL_INCH_TIGHT
            || endAngError.absoluteValue > TURNING_TOL_DEG_TIGHT / 360.0 * TAU
        )

        drivebase.controlMotors(0.0, 0.0, 0.0)

        drivebase.resetMotorEncoders()
    }
}