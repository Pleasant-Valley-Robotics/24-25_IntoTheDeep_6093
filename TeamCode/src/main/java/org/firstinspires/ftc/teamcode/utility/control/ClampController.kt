package org.firstinspires.ftc.teamcode.utility.control

class ClampController(
    private val pGain: Double,
    private val maxControl: Double,
) : ErrorController {
    override fun accept(error: Double): Double = (error * pGain).coerceIn(-maxControl, maxControl)
}