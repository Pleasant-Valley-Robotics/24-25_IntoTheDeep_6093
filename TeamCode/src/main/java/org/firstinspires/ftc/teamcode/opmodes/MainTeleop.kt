package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.systems.Drivebase
import org.firstinspires.ftc.teamcode.systems.Extender
import org.firstinspires.ftc.teamcode.systems.Lift
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")
        val drivebase = Drivebase(hardwareMap)
        val lift = Lift(hardwareMap)
        val extender = Extender(hardwareMap)

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            val driving = launch {
                while (isActive) {
                    yield()

                    val xInput = gamepad1.left_stick_x.toDouble()
                    val yInput = -gamepad1.left_stick_y.toDouble()
                    val turnInput = gamepad1.right_stick_x.toDouble()

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    val liftInput = -gamepad2.left_stick_y.toDouble()
                    val extendInput = -gamepad2.left_stick_x.toDouble()

                    val inLimitUpper = lift.liftHeight <= LiftConstants.MAX_LIFT_HEIGHT_INCH
                    // 0.3 because of slight drifts causing error
                    val inLimitLower = lift.liftHeight >= 0.3

                    lift.liftPower =
                        if (liftInput > 0 && inLimitUpper || liftInput < 0 && inLimitLower) liftInput
                        else 0.0

                    extender.extendMotor.power = extendInput
                }
            }


            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)

                telemetry.status("running")

                yield()
            }

            driving.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.liftPower = 0.0
            extender.extendMotor.power = 0.0
        }

    }
}