package org.firstinspires.ftc.teamcode.opmodes

import androidx.appcompat.app.ActionBarDrawerToggle.Delegate
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelChildren
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.systems.Drivebase
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

fun Telemetry.status(status: String) {
    addData("status", status)
    update()
}

class RisingEdgeDetector(val buttonState: () -> Boolean) {
    private var wasPressed = false
    private var on = false

    fun get(): Boolean {
        val nowPressed = buttonState()
        if (nowPressed && !wasPressed) on = !on
        wasPressed = nowPressed

        return on
    }

    fun set(value: Boolean) {
        on = value
    }
}

suspend fun parallelWait(vararg tasks: suspend () -> Unit) = coroutineScope {
    for (task in tasks) launch { task() }
}

suspend fun parallelRace(vararg tasks: suspend () -> Unit) = coroutineScope {
    var anyDone = false
    for (task in tasks) launch { task(); anyDone = true }

    while (!anyDone) yield()

    this.coroutineContext.cancelChildren()
}

suspend fun moveToBasket(drivebase: Drivebase, maxSpeed: Double = 0.5) =
    drivebase.driveOffsetGlobal(
        xInches = -23.22,
        yInches = 7.938,
        angleRadians = 0.8,
        maxPower = maxSpeed,
        precise = false,
    )
