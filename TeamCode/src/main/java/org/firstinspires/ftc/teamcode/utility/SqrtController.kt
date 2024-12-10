package org.firstinspires.ftc.teamcode.utility

import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.math.sqrt

class SqrtController(
    maxError: Double,
    private val maxControl: Double,
) : ErrorController {
    private val pGain = maxControl / sqrt(maxError)

    override fun accept(error: Double): Double = (pGain * sqrt(error.absoluteValue) * error.sign).coerceIn(-maxControl, maxControl)
}