package org.firstinspires.ftc.teamcode.utility.control

/**
 * a feedforward pid controller, implemented using
 * [this article](https://en.wikipedia.org/wiki/Proportional–integral–derivative_controller).
 * @param kp scalar on error
 * @param ti integral time
 * @param td derivative time
 * @param clamp max value for integrator. if `null` no clamp is applied. integrator clamped into `[-x, x]`
 * @param maxValue maximum value for entire system. output clamped into `[-x, x]`
 * @param feedforward a provider for feedforward.
 */
class PidfController(
    private val kp: Double,
    ti: Double,
    td: Double,
    private val clamp: Double?,
    private val maxValue: Double,
    private val feedforward: () -> Double,
) {
    private val ki = kp / ti
    private val kd = kp * td

    var setpoint = 0.0

    private var lastError = 0.0
    private var integral = 0.0

    fun update(value: Double, deltaTime: Double): Double {
        val error = setpoint - value

        val prop = error * kp
        integral += error * deltaTime * ki
        if (clamp != null) integral = integral.coerceIn(-clamp, clamp)
        val dv = (error - lastError) / deltaTime * kd

        lastError = error

        val closedLoop = prop + integral + dv

        val openLoop = feedforward()

        return (closedLoop + openLoop).coerceIn(-maxValue, maxValue)
    }
}