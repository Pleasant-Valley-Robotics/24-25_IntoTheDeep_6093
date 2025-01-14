package org.firstinspires.ftc.teamcode.utility.vision

import org.firstinspires.ftc.teamcode.utility.vision.ColorFilter.FilterParams

enum class BlockColor {
    Red,
    Yellow,
    Blue;

    fun getFilterParams(): FilterParams = when (this) {
        Red -> FilterParams(
            minA = 148,
            maxA = 201,
            minB = 121,
            maxB = 158,
            aPerB = -0.4116666666666666f,
            bPerA = 0.6566666666666667f,
        )

        Yellow -> FilterParams(
            minA = 107,
            maxA = 150,
            minB = 138,
            maxB = 201,
            aPerB = 0.03500000000000014f,
            bPerA = 0.2616666666666667f,
        )

        Blue -> FilterParams(
            minA = 131,
            maxA = 204,
            minB = 28,
            maxB = 96,
            aPerB = -0.6416666666666666f, 
            bPerA = -0.0016666666666664831f,
        )
    }
}
