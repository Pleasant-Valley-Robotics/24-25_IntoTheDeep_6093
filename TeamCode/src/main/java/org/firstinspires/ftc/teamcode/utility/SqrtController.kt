package org.firstinspires.ftc.teamcode.utility

import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.sqrt

class SqrtController(
    private val pGain: Double,
    private val maxControl: Double,
) : ErrorController {

    override fun accept(error: Double): Double = (pGain * sqrt(error.absoluteValue) * error.sign).coerceIn(-maxControl, maxControl)
}