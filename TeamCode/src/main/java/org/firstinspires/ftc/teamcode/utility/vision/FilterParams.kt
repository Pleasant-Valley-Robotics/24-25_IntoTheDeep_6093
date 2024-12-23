package org.firstinspires.ftc.teamcode.utility.vision

data class FilterParams(
    val aMin: Int,
    val aMax: Int,
    val bMin: Int,
    val bMax: Int,
    val aPerB: Double,
    val bPerA: Double,
)
