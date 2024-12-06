package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_IN_POS
import org.firstinspires.ftc.teamcode.utility.FlipperConstants.FLIPPER_OUT_POS

class Flipper(hardwareMap: HardwareMap) {
    private val flipperServo = hardwareMap.servo.get("BucketPivot").apply {
        position = 1.0
    }

    fun pivotTo(out: Boolean) {
        flipperServo.position = if (out) FLIPPER_OUT_POS else FLIPPER_IN_POS
    }
}