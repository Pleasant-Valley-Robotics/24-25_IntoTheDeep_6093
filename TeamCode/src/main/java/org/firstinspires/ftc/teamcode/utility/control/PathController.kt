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
import kotlin.math.absoluteValue

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
            val nearest = trajectory.nearestPoint(currentPose)
            val targets = trajectory.pointsAround(currentPose, lookahead)
            setPoint = targets.sorted().find { it > setPoint }
                       ?: endPoint.takeIf { (trajectory.getPos(it) - trajectory.getPos(nearest)).pos.length < lookahead }
                       ?: nearest

            val (posError, angError) = trajectory.getPos(setPoint) - currentPose
            val (xError, yError) = posError.rotate(-currentPose.angRad)

            val (endPosError, endAngError) = trajectory.getPos(endPoint) - currentPose

            val xInput = xPID.accept(xError)
            val yInput = yPID.accept(yError)
            val angInput = angPID.accept(angError)

            val (vel, angVel) = trajectory.getVel(endPoint)
            val (xVel, yVel) = vel

            drivebase.controlMotors(xInput + xVel, yInput + yVel, angInput + angVel)

            yield()
        } while (
            setPoint != endPoint
            || endPosError.length > MOVEMENT_TOL_INCH_TIGHT
            || endAngError.absoluteValue > TURNING_TOL_DEG_TIGHT
        )

        drivebase.resetMotorEncoders()
    }
}