package org.firstinspires.ftc.teamcode.opmodes

import kotlinx.coroutines.cancelChildren
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.yield
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.systems.Clipper
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.RightLift
import org.firstinspires.ftc.teamcode.utility.LiftConstants.MAX_LIFT_HEIGHT_RIGHT
import kotlin.math.PI

var LAST_AUTO_START_POS: Pose2D? = null

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

suspend fun cancelWith(supplier: () -> Boolean, task: suspend () -> Unit) = coroutineScope {
    parallelRace(task, { while (!supplier()) yield() })
}

suspend fun moveToBasket(drivebase: Drivebase, maxSpeed: Double = 0.5) =
    drivebase.driveToPositionGlobal(
        xInches = -23.22,
        yInches = 7.938,
        angleRadians = 0.8,
        maxPower = maxSpeed,
        precise = false,
    )

suspend fun moveToRungs(drivebase: Drivebase, maxSpeed: Double = 0.5) =
    drivebase.driveToPositionGlobal(
        xInches = 33.4708,
        yInches = 28.1057,
        angleRadians = PI / 2,
        maxPower = maxSpeed,
        precise = false,
    )

suspend fun moveToSubLeft(drivebase: Drivebase, maxSpeed: Double = 0.5) =
    drivebase.driveToPositionGlobal(
        xInches = 8.3147,
        yInches = 53.5766,
        angleRadians = 0.0,
        maxPower = maxSpeed,
        precise = false,
    )

suspend fun moveToSubRight(drivebase: Drivebase, maxSpeed: Double = 0.5) =
    drivebase.driveToPositionGlobal(
        xInches = 54.7722,
        yInches = 53.5766,
        angleRadians = PI,
        maxPower = maxSpeed,
        precise = false,
    )

suspend fun scoreSample(
    drivebase: Drivebase,
    lift: RightLift,
    clipper: Clipper,
    param: Double,
) {
    val xParam = (1 - param) * 20.4613 + param * 44.8
    parallelWait(
        {
            drivebase.driveToPositionGlobal(xParam, 15.58595, -PI / 2, 1.0, false)
            drivebase.driveToPositionGlobal(xParam, 25.58595, -PI / 2, 0.5, true)
        },
        { lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 1.8) },
    )

    lift.moveLiftTo(MAX_LIFT_HEIGHT_RIGHT - 7.0)
    clipper.moveClaw(Clipper.ClipperState.Open)
}

suspend fun moveToSampleParam(
    drivebase: Drivebase,
    param: Double,
) {
    val xParam = (1 - param) * 20.4613 + param * 44.8
    drivebase.driveToPositionGlobal(xParam, 15.58595, -PI / 2, 1.0, false)
    drivebase.driveToPositionGlobal(xParam, 25.58595, -PI / 2, 0.5, true)
}

suspend fun pickClip(
    drivebase: Drivebase,
    lift: RightLift,
    clipper: Clipper
) {
    //clip pos (69.3698, -1.9069, 1.5408)
    parallelWait({
        drivebase.driveToPositionGlobal(69.3698, 4.9069, PI / 2, 1.0, false)
        drivebase.driveToPositionGlobal(69.3698, -1.2069, PI / 2, 1.0, false)
    },
        { lift.moveLiftTo(0.0) }
    )

    clipper.moveClaw(Clipper.ClipperState.Closed)
    delay(250L)
}
