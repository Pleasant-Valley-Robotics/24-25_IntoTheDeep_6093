package org.firstinspires.ftc.teamcode.utility.vision

enum class BlockColor {
    Red,
    Yellow,
    Blue;

    fun getFilterParams(): FilterParams = when (this) {
        Red -> FilterParams(
            aMin = 148,
            aMax = 196,
            bMin = 138,
            bMax = 179,
            aPerB = 0.0016666666666667052,
            bPerA = 0.6566666666666667
        )
        Yellow -> FilterParams(
            aMin = 113,
            aMax = 147,
            bMin = 149,
            bMax = 196,
            aPerB = 0.04833333333333334,
            bPerA = 0.44666666666666677
        )
        Blue -> FilterParams(
            aMin = 142,
            aMax = 170,
            bMin = 58,
            bMax = 115,
            aPerB = -0.5416666666666666,
            bPerA = -0.0016666666666664831
        )
    }
}