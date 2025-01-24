package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.ClipperConstants

class Clipper(hardwareMap: HardwareMap) {
    private val clipperServo = hardwareMap.servo.get("Clipper")!!

    enum class ClipperState {
        Open,
        Closed,
    }

    fun moveClaw(state: ClipperState) {
        clipperServo.position = when (state) {
            ClipperState.Open -> ClipperConstants.CLIPPER_OPEN_POS
            ClipperState.Closed -> ClipperConstants.CLIPPER_CLOSED_POS
        }
    }
}