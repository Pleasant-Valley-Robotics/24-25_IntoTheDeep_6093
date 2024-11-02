package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Winch(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.dcMotor.get("Winch").apply {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

    }

    /**
     * moves the winch motor.
     *
     * @param power how fast to move the motor. positive is up. `[-1, 1]`
     */
    fun moveMotor(power: Double) {
        motor.power = power
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("winch position", motor.currentPosition)
    }
}