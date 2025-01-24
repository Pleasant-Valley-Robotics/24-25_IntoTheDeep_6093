package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap

/** thing with 2 grippy wheels that hand off to the flipper */
class Spintake(hardwareMap: HardwareMap) {
    private val spinLeft = hardwareMap.crservo.get("LIntake")
    private val spinRight = hardwareMap.crservo.get("RIntake")

    enum class SpintakeState {
        Spit,
        Suck,
        Off,
    }

    /**
     * sets the wheel intake state
     *
     * @param spintakeState what state the wheels should be in.
     */
    fun controlIntakeState(spintakeState: SpintakeState) {
        val power = when (spintakeState) {
            SpintakeState.Spit -> -1.0
            SpintakeState.Suck -> 1.0
            SpintakeState.Off -> 0.0
        }

        // negative clawLeft out
        // positive clawRight out
        spinLeft.power = -power
        spinRight.power = power
    }

    /**
     * directly controls both servo motors. negative is ccw.
     *
     * @param leftPower speed of left motor. `[-1, 1]`
     * @param rightPower speed of right motor. `[-1, 1]`
     */
    fun controlIntakeDirect(leftPower: Double, rightPower: Double) {
        spinLeft.power = leftPower
        spinRight.power = rightPower
    }
}