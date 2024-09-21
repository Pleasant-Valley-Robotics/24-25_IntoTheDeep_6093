package org.firstinspires.ftc.teamcode.opmodes

import org.firstinspires.ftc.robotcore.external.Telemetry

fun Telemetry.status(status: String) {
    addData("Status", status)
    update()
}
