package org.firstinspires.ftc.teamcode.utility.control

import kotlinx.coroutines.yield
import kotlin.math.absoluteValue

interface ErrorController {
    fun accept(error: Double): Double

    suspend fun controlThing(
        tolerance: Double,
        error: () -> Double,
        output: (Double) -> Unit,
    ) {
        do {
            val delta = error()
            output(this.accept(delta))

            yield()
        } while (delta.absoluteValue > tolerance)
    }
}