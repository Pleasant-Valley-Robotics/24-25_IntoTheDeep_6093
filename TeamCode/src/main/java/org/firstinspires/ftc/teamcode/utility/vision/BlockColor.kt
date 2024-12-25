package org.firstinspires.ftc.teamcode.utility.vision

import org.firstinspires.ftc.teamcode.utility.vision.ColorFilter.FilterParams

enum class BlockColor {
    Red,
    Yellow,
    Blue;

    fun getFilterParams(): FilterParams = when (this) {
        Red -> FilterParams(
            minA = 148,
            maxA = 196,
            minB = 138,
            maxB = 179,
            aPerB = 0.0016666667f,
            bPerA = 0.6566667f,
        )

        Yellow -> FilterParams(
            minA = 113,
            maxA = 147,
            minB = 149,
            maxB = 196,
            aPerB = 0.048333332f,
            bPerA = 0.44666666f,
        )

        Blue -> FilterParams(
            minA = 142,
            maxA = 170,
            minB = 58,
            maxB = 115,
            aPerB = -0.5416667f,
            bPerA = -0.0016666667f,
        )
    }
}