package org.firstinspires.ftc.teamcode.utility.vision

import org.firstinspires.ftc.teamcode.utility.vision.ColorFilter.FilterParams

enum class BlockColor {
    Red,
    Yellow,
    Blue;

    fun getFilterParams() = when (this) {
        Red -> FilterParams(
            minA = 148,
            maxA = 201,
            minB = 121,
            maxB = 158,
            aPerB = -0.41166666f,
            bPerA = 0.6566667f,
        )

        Yellow -> FilterParams(
            minA = 107,
            maxA = 150,
            minB = 138,
            maxB = 201,
            aPerB = 0.035f,
            bPerA = 0.26166666f,
        )

        Blue -> FilterParams(
            minA = 131,
            maxA = 204,
            minB = 28,
            maxB = 96,
            aPerB = -0.64166665f,
            bPerA = -0.0016666667f,
        )
    }
}
