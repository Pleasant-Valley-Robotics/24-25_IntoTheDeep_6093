package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "Motor Tests")
class MotorTesting : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("Initializing")

        val motorNames = listOf("FLDrive", "FRDrive", "BLDrive", "BRDrive")
        val encoderNames = listOf("FLEncoder", "FREncoder", "BLEncoder", "BREncoder")

        val motors = motorNames.map(hardwareMap.dcMotor::get)

        motors.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.direction = DcMotorSimple.Direction.FORWARD
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        val buttons = listOf(
            { gamepad1.triangle },
            { gamepad1.circle },
            { gamepad1.square },
            { gamepad1.cross },
        )

        telemetry.status("Initialized")

        waitForStart()

        while (opModeIsActive()) {
            for (i in motors.indices) {
                motors[i].power = if (buttons[i]()) 1.0 else 0.0

                telemetry.addData(motorNames[i], motors[i].power)
                telemetry.addData(encoderNames[i], motors[i].currentPosition)
            }

            telemetry.status("Running")
        }

        telemetry.status("Finished")
    }

}