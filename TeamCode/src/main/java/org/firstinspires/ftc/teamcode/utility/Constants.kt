package org.firstinspires.ftc.teamcode.utility

import kotlin.math.PI

object DriveConstants {
    private const val WHEEL_DIAMETER = 4

    // NeveRest 20 gearmotors with a 7:9 gearing on top
    // note that NeveRest 20s are actually a 19.2:1 gear ratio
    // https://www.andymark.com/products/neverest-orbital-20-gearmotor
    private const val TICKS_PER_REV = 537.6 * 7 / 9
    const val ENCODER_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * PI)
    const val STRAFING_CORRECTION = 1.1
}

object LiftConstants {
    const val ENCODER_PER_INCH = 0.0
    const val MAX_LIFT_HEIGHT_INCH = 0.0
}

object PrototypeConstants {
    const val MAX_LIFT_HEIGHT = 0.3409
}

