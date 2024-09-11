package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val drivebase = Drivebase(hardwareMap)

        telemetry.status("Initialized")

        waitForStart()


        while (opModeIsActive()) {
            val xInput = gamepad1.left_stick_x.toDouble()
            val yInput = -gamepad1.left_stick_y.toDouble()
            val turnInput = gamepad1.right_stick_x.toDouble()

            drivebase.controlMotors(xInput, yInput, turnInput)

            drivebase.addTelemetry(telemetry)
            telemetry.status("Running")
        }
    }
}