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

    // 1 inch of error should be 0.2 power
    const val DRIVING_P_GAIN = 0.2 / 1.0
    const val STRAFING_P_GAIN = 0.2 / 1.0

    const val MOVEMENT_TOL_INCH = 0.3

    const val TURNING_TOL_DEG = 2.0

    // 20 degrees of error should be 0.1 power
    const val TURNING_P_GAIN = 0.1 / 15.0
}

object LiftConstants {
    // 5203 gobilda 26.9:1s
    private const val TICKS_PER_REV = 26.85123966942149 * BASE_PPR

    // 2mm pitch gt2 hub mount, 38.2 pitch diameter
    private const val PITCH_DIAMETER = 38.2 * (0.1 / 2.54)

    const val ENCODER_PER_INCH = TICKS_PER_REV / (PITCH_DIAMETER * PI)
    const val MAX_LIFT_HEIGHT_INCH = 15.0
    const val MIN_LIFT_HEIGHT_INCH = 0.5
}

object ExtenderConstants {
    const val ENCODER_PER_INCH = LiftConstants.ENCODER_PER_INCH
    const val MAX_EXTENSION_INCH = 19.0
    const val MIN_EXTENSION_INCH = 0.7
}

object SpintakeConstants {
    // positive is down
    const val PIVOT_UP_POS = 0.0
    const val PIVOT_DODGE_POS = 0.2
    const val PIVOT_DOWN_POS = 1.0
    const val SERVO_VEL_ENC_S = (PIVOT_DOWN_POS - PIVOT_UP_POS) / 1.25
}

object FlipperConstants {
    const val FLIPPER_IN_POS = 1.0
    const val FLIPPER_OUT_POS = 0.4
    const val SERVO_VEL_ENC_S = (FLIPPER_IN_POS - FLIPPER_OUT_POS) / 0.8
}
