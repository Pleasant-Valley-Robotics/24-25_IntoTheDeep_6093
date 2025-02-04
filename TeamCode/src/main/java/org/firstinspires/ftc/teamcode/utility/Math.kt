package org.firstinspires.ftc.teamcode.utility

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

fun rotate(point: Pair<Double, Double>, angleRadians: Double) = Pair(
    cos(angleRadians) * point.first - sin(angleRadians) * point.second,
    sin(angleRadians) * point.first + cos(angleRadians) * point.second,
)

fun dist(point: Pair<Double, Double>) = point.run {
    sqrt(first * first + second * second)
}

const val TAU = PI * 2