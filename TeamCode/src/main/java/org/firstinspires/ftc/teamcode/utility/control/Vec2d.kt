package org.firstinspires.ftc.teamcode.utility.control

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Vec2d(
    val x: Double,
    val y: Double,
) {
    operator fun unaryPlus() = this
    operator fun unaryMinus() = this * -1.0
    operator fun plus(other: Vec2d) = Vec2d(x + other.x, y + other.y)
    operator fun minus(other: Vec2d) = this + -other
    operator fun times(other: Vec2d) = x * other.x + y * other.y
    operator fun times(scalar: Double) = Vec2d(x * scalar, y * scalar)
    operator fun div(scalar: Double) = this * (1.0 / scalar)

    fun rotate(angle: Double) = Vec2d(
        cos(angle) * x - sin(angle) * y,
        sin(angle) * x + cos(angle) * y,
    )

    val length = sqrt(this * this)
}
