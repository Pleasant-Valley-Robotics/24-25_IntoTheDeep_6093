package org.firstinspires.ftc.teamcode.utility.control

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D

data class PoseData(
    val pos: Vec2d,
    val angRad: Double,
) {
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
