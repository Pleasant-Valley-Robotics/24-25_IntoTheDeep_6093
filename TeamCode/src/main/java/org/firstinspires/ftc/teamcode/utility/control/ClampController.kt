package org.firstinspires.ftc.teamcode.utility.control

class ClampController(
    val pGain: Double,
    var maxValue: Double,
) : ErrorController {
    override fun accept(error: Double): Double = (error * pGain).coerceIn(-maxValue, maxValue)
}