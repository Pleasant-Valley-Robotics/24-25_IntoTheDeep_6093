package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_IN_POS
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_OUT_POS

/** the thing on the lift that scores samples in the basket */
class Flipper(hardwareMap: HardwareMap) {
    private val flipperServo = hardwareMap.servo.get("BucketPivot").apply {
        position = FLIPPER_IN_POS
    }

    /**
     * moves the flipper out or in
     *
     * @param out if `true`, moves flipper out. otherwise, moves flipper in.
     */
    fun pivotTo(out: Boolean) {
        flipperServo.position = if (out) FLIPPER_OUT_POS else FLIPPER_IN_POS
    }
}