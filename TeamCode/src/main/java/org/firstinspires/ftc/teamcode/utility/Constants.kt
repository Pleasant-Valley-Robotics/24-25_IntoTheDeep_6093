package org.firstinspires.ftc.teamcode.utility

import kotlin.math.PI

// note that we should be using inches, seconds, and degrees unless specified otherwise

private const val BASE_PPR = 28

object DriveConstants {
    // dan wheel diameter and ticks
    // private const val WHEEL_DIAMETER = 4
    // NeveRest 20 gearmotors with a 7:9 gearing on top
    // https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // private const val TICKS_PER_REV = 537.6 * 7 / 9

    // 5203 gobilda 19.2:1s
    private const val TICKS_PER_REV = 19.20320855614973 * BASE_PPR

    // gobilda 140mm wheels as inches
    private const val WHEEL_DIAMETER = 140 * (0.1 / 2.54)


    const val ENCODER_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * PI)
    const val STRAFING_CORRECTION = 1.1
}

object LiftConstants {
    // 5203 gobilda 26.9:1s
    private const val TICKS_PER_REV = 26.85123966942149 * BASE_PPR

    // 2mm pitch gt2 hub mount, 38.2 pitch diameter
    private const val PITCH_DIAMETER = 38.2 * (0.1 / 2.54)

    const val ENCODER_PER_INCH = TICKS_PER_REV / (PITCH_DIAMETER * PI)
    const val MAX_LIFT_HEIGHT_INCH = 15.0
}

object PrototypeConstants {
    const val MAX_LIFT_HEIGHT = 0.3409
}

