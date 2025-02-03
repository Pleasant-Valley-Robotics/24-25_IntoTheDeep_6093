package org.firstinspires.ftc.teamcode.utility.control

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import kotlin.math.PI

/**
 * wraps an angle in radians to the range `[-π, π]`
 * @param n angle to wrap
 * @return the radian value
 */
private fun wrapRadians(n: Double) = (n + PI).mod(PI * 2) - PI

data class PoseData(
    val pos: Vec2d,
    val angRad: Double,
) {
    operator fun unaryPlus() = this
    operator fun unaryMinus() = PoseData(-pos, -angRad)
    operator fun plus(other: PoseData) = PoseData(
        pos + other.pos,
        wrapRadians(angRad + other.angRad)
    )

    operator fun minus(other: PoseData) = this + -other

    constructor(pose2D: Pose2D) : this(
        Vec2d(
            pose2D.getX(DistanceUnit.INCH),
            pose2D.getY(DistanceUnit.INCH),
        ),
        pose2D.getHeading(AngleUnit.RADIANS),
    )

    val pose2d = Pose2D(
        DistanceUnit.INCH, pos.x, pos.y,
        AngleUnit.RADIANS, angRad,
    )
}
