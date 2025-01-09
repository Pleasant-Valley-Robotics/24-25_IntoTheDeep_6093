package org.firstinspires.ftc.teamcode.utility.control

import kotlin.math.absoluteValue
import kotlin.math.withSign

class BangBangController(
    val speed: Double,
    val thresh: Double,
) : ErrorController {
    override fun accept(error: Double): Double =
        if (error.absoluteValue > thresh) 0.0
        else speed.withSign(error)
}