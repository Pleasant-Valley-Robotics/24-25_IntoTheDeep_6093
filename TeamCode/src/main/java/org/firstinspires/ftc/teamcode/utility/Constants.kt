package org.firstinspires.ftc.teamcode.utility

import kotlin.math.PI

// note that we should be using inches, seconds, and degrees unless specified otherwise

object DriveConstants {
    // dan wheel diameter and ticks
    // private const val WHEEL_DIAMETER = 4
    // NeveRest 20 gearmotors with a 7:9 gearing on top
    // https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // private const val TICKS_PER_REV = 537.6 * 7 / 9

    // gobilda 144mm wheels as inches
    private const val WHEEL_DIAMETER = 144 * (0.1 / 2.54)

    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-24mm-length-8mm-rex-shaft-223-rpm-3-3-5v-encoder/
    // PPR at output shaft, no gear ratio
    private const val TICKS_PER_REV = 751.8

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

