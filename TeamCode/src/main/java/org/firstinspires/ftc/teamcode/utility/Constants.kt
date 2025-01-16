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

    // max gain = first gain that causes oscillations
    // max time = how long each one of those oscillations takes
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    private const val D_GAIN = 0.2
    private const val D_TIME = .51
    private const val S_GAIN = 0.3
    private const val S_TIME = .6
    private const val T_GAIN = 2.7
    private const val T_TIME = .23

    const val DRIVING_P_GAIN = 0.33 * D_GAIN
    const val DRIVING_I_GAIN = 0.66 * D_GAIN / D_TIME
    const val DRIVING_D_GAIN = 0.11 * D_GAIN * D_TIME

    const val STRAFING_P_GAIN = 0.33 * S_GAIN
    const val STRAFING_I_GAIN = 0.66 * S_GAIN / S_TIME
    const val STRAFING_D_GAIN = 0.11 * S_GAIN * S_TIME

    const val TURNING_P_GAIN = 0.33 * T_GAIN
    const val TURNING_I_GAIN = 0.66 * T_GAIN / T_TIME
    const val TURNING_D_GAIN = 0.11 * T_GAIN * T_TIME

    const val MOVEMENT_TOL_INCH = 0.2
    const val TURNING_TOL_DEG = 5.0
}

object LiftConstants {
    // 5203 gobilda 26.9:1s
    private const val TICKS_PER_REV = 26.85123966942149 * BASE_PPR

    // 2mm pitch gt2 hub mount, 38.2 pitch diameter
    private const val PITCH_DIAMETER = 38.2 * (0.1 / 2.54)

    const val ENCODER_PER_INCH = TICKS_PER_REV / (PITCH_DIAMETER * PI)
    const val MAX_LIFT_HEIGHT_INCH = 36.0
    const val MIN_LIFT_HEIGHT_INCH = 0.5
}

object ExtenderConstants {
    const val ENCODER_PER_INCH = LiftConstants.ENCODER_PER_INCH
    const val MAX_EXTENSION_INCH = 19.0
    const val MIN_EXTENSION_INCH = 0.7
}

object SpintakeConstants {
    // positive is down
    const val PIVOT_UP_POS = 1.0
    const val PIVOT_DODGE_POS = 0.6
    const val PIVOT_DOWN_POS = 0.0

    const val PIVOT_LOOK_POS = 0.30

    const val SERVO_VEL_ENC_S = (PIVOT_DOWN_POS - PIVOT_UP_POS) / 1.25
}

object FlipperConstants {
    const val FLIPPER_IN_POS = 1.0
    const val FLIPPER_OUT_POS = 0.4
    const val SERVO_VEL_ENC_S = (FLIPPER_IN_POS - FLIPPER_OUT_POS) / 0.8
}

object CameraConstants {
    const val PIVOT_HEIGHT_IN = 5.625
    const val BLOCK_HEIGHT_IN = 1.5

    const val CAMERA_RADIUS_IN = 2.875
    const val CAMERA_OFFSET_Y_IN = 0.7086614173228347
    const val CAMERA_OFFSET_X_IN = 1.6

    const val PIVOT_DOWN_ANGLE_RAD = Math.PI / 4
    const val PIVOT_UP_ANGLE_RAD = 0.0

    const val TARGET_BLOCK_OFFSET_IN = 1.0

    const val Y_CORRECT_P = 0.2
    const val Y_CORRECT_MAX = 0.3
    const val Y_CORRECT_THRESH_IN = 0.2

    const val X_CORRECT_SPEED = 0.6
    const val X_CORRECT_THRESH_IN = 0.2
}
