package org.firstinspires.ftc.teamcode.utility

import org.firstinspires.ftc.teamcode.utility.MotorConstants.BELT_PITCH_DIAMETER
import org.firstinspires.ftc.teamcode.utility.MotorConstants.TICKS_PER_REV_19_2TO1
import org.firstinspires.ftc.teamcode.utility.MotorConstants.TICKS_PER_REV_26_9TO1
import org.firstinspires.ftc.teamcode.utility.MotorConstants.WHEEL_DIAMETER
import kotlin.math.PI

// note that we should be using inches, seconds, and degrees unless specified otherwise


object MotorConstants {
    private const val BASE_PPR = 28
    // dan wheel diameter and ticks
    // private const val WHEEL_DIAMETER = 4
    // NeveRest 20 gearmotors with a 7:9 gearing on top
    // https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // private const val TICKS_PER_REV = 537.6 * 7 / 9

    // 5203 gobilda 19.2:1s
    const val TICKS_PER_REV_19_2TO1 = 19.20320855614973 * BASE_PPR

    // 5203 gobilda 26.9:1s
    const val TICKS_PER_REV_26_9TO1 = 26.85123966942149 * BASE_PPR

    // 2mm pitch gt2 hub mount, 38.2 pitch diameter
    const val BELT_PITCH_DIAMETER = 38.2 * (0.1 / 2.54)

    // gobilda 140mm wheels as inches
    const val WHEEL_DIAMETER = 140 * (0.1 / 2.54)
}

object DrivebaseConstants {
    const val ENCODER_PER_INCH = TICKS_PER_REV_19_2TO1 / (WHEEL_DIAMETER * PI)
    const val STRAFING_CORRECTION = 1.1

    // max gain = first gain that causes oscillations
    // max time = how long each one of those oscillations takes
    // https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
    private const val D_GAIN = 0.3
    private const val D_TIME = .51 * 2
    private const val S_GAIN = 0.4
    private const val S_TIME = .6 * 2
    private const val T_GAIN = 1.7
    private const val T_TIME = .7 * 2

    const val DRIVING_P_GAIN = 0.33 * D_GAIN
    const val DRIVING_I_GAIN = 0.66 * D_GAIN / D_TIME
    const val DRIVING_D_GAIN = 0.11 * D_GAIN * D_TIME

    const val STRAFING_P_GAIN = 0.33 * S_GAIN
    const val STRAFING_I_GAIN = 0.66 * S_GAIN / S_TIME
    const val STRAFING_D_GAIN = 0.11 * S_GAIN * S_TIME

    const val TURNING_P_GAIN = 0.33 * T_GAIN
    const val TURNING_I_GAIN = 0.66 * T_GAIN / T_TIME
    const val TURNING_D_GAIN = 0.11 * T_GAIN * T_TIME

    const val MOVEMENT_TOL_INCH_LOOSE = 0.5
    const val TURNING_TOL_DEG_LOOSE = 4.0
    const val MOVEMENT_TOL_INCH_TIGHT = 0.1
    const val TURNING_TOL_DEG_TIGHT = 2.0
}

object LiftConstants {
    const val ENCODER_PER_INCH = TICKS_PER_REV_26_9TO1 / (BELT_PITCH_DIAMETER * PI)
    const val MAX_LIFT_HEIGHT_LEFT = 36.0
    const val MAX_LIFT_HEIGHT_RIGHT = 18.0
    const val MIN_LIFT_HEIGHT = 0.5
}

object ExtenderConstants {
    const val ENCODER_PER_INCH = TICKS_PER_REV_26_9TO1 / (BELT_PITCH_DIAMETER * PI)
    const val MAX_EXTENSION = 19.0
    const val MIN_EXTENSION = 0.7
}

object SpintakeConstants {
    // positive is down
    const val SPINTAKE_UP_POS = 1.0
    const val SPINTAKE_DODGE_POS = 0.6
    const val SPINTAKE_DOWN_POS = 0.0

    const val SPINTAKE_LOOK_POS = 0.30
}

object BucketConstants {
    const val BUCKET_IN_POS = 1.0
    const val BUCKET_OUT_POS = 0.4
    const val BUCKET_TOUCH_POS = 0.0
}

object CameraConstants {
    const val SPINTAKE_HEIGHT_IN = 5.625
    const val BLOCK_HEIGHT_IN = 1.5

    const val CAMERA_RADIUS_IN = 2.875
    const val CAMERA_OFFSET_Y_IN = 0.7086614173228347
    const val CAMERA_OFFSET_X_IN = 1.6

    const val SPINTAKE_DOWN_ANGLE_RAD = Math.PI / 4
    const val SPINTAKE_UP_ANGLE_RAD = 0.0

    const val TARGET_BLOCK_OFFSET_IN = 1.0

    const val Y_CORRECT_P = 0.2
    const val Y_CORRECT_MAX = 0.3
    const val Y_CORRECT_THRESH_IN = 0.2

    const val X_CORRECT_SPEED = 0.6
    const val X_CORRECT_THRESH_IN = 0.2
}
