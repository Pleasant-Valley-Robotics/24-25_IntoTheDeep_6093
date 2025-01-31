package org.firstinspires.ftc.teamcode.utility.control

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * a feedforward pid controller, implemented using
 * [this article](https://en.wikipedia.org/wiki/Proportional–integral–derivative_controller).
 * @param kp scalar on error
 * @param ki integral time
 * @param kd derivative time
 * @param clamp max value for integrator. if `null` no clamp is applied. integrator clamped into `[-x, x]`
 * @param maxValue maximum value for entire system. output clamped into `[-x, x]`
 * @param derivativeGetter replaces the internal finite differences calculation for derivative if supplied.
 */
class PidController(
    private val kp: Double,
    private val ki: Double = 0.0,
    private val kd: Double = 0.0,
    private val clamp: Double?,
    private val maxValue: Double,
    private val derivativeGetter: (() -> Double)? = null,
) : ErrorController {
    private val timer = ElapsedTime()
    private var lastError = 0.0
    private var integral = 0.0

    override fun accept(error: Double): Double {
        val deltaTime = timer.seconds()
        timer.reset()

        val proportional = error * kp
        if (error * lastError < 0.0) integral *= -0.25
        integral += error * deltaTime * ki
        if (clamp != null) integral = integral.coerceIn(-clamp, clamp)
        val errorRate = derivativeGetter?.invoke() ?: ((error - lastError) / deltaTime)
        val derivative = errorRate * kd

        lastError = error

        val closedLoop = proportional + integral + derivative

        return closedLoop.coerceIn(-maxValue, maxValue)
    }
}