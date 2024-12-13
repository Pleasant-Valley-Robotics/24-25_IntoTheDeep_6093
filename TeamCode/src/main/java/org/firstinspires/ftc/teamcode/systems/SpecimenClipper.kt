package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap

class SpecimenClipper(hardwareMap: HardwareMap) {
    private val clipperServo = hardwareMap.servo.get("Clipper")!!.apply {
        this.position = 1.0
    }

    enum class ClipperState {
        Open,
        Closed,
    }

    fun moveClaw(state: ClipperState) {
        clipperServo.position = when (state) {
            // TODO: change position constants
            ClipperState.Open -> 1.0
            ClipperState.Closed -> 0.0
        }
    }
}