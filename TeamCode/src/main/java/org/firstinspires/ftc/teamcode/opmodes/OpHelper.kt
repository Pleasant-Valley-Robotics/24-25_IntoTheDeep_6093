package org.firstinspires.ftc.teamcode.opmodes

import kotlinx.coroutines.Job
import kotlinx.coroutines.cancelChildren
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry

fun Telemetry.status(status: String) {
    addData("Status", status)
    update()
}

fun onRisingEdge(buttonState: () -> Boolean, callback: (Boolean) -> Unit): () -> Unit {
    var lastPressed = false
    var on = false

    return {
        val nowPressed = buttonState()
        if (nowPressed && !lastPressed) {
            on = !on
        }

        callback(on)

        lastPressed = nowPressed
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
