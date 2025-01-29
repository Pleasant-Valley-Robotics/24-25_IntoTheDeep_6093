package org.firstinspires.ftc.teamcode.opmodes

import androidx.appcompat.app.ActionBarDrawerToggle.Delegate
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelChildren
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.properties.ReadWriteProperty
import kotlin.reflect.KProperty

fun Telemetry.status(status: String) {
    addData("Status", status)
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

class WorkGroup private constructor(
    private val tasks: MutableList<Job>,
) {
    fun just(job: Job): WorkGroup {
        return WorkGroup(mutableListOf(job))
    }

    fun work() {

    }
}

// work({}, {}, {})
