package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.Telemetry

fun Telemetry.status(status: String) {
    addData("Status", status)
    update()
}
