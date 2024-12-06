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
import org.firstinspires.ftc.teamcode.systems.Spintake
import org.firstinspires.ftc.teamcode.utility.ExtenderConstants
import org.firstinspires.ftc.teamcode.utility.LiftConstants

@TeleOp(name = "MainTeleop")
class MainTeleop : LinearOpMode() {
    override fun runOpMode() {
        telemetry.status("initializing motors")
        val drivebase = Drivebase(hardwareMap)
        val lift = Lift(hardwareMap)
        val extender = Extender(hardwareMap)
        val spintake = Spintake(hardwareMap)

        telemetry.status("initialized")

        waitForStart()

        runBlocking {
            val driving = launch {
                var pivotInputActive = false
                var overriding = false

                while (isActive) {
                    yield()

                    val xInput = gamepad1.left_stick_x.toDouble()
                    val yInput = -gamepad1.left_stick_y.toDouble()
                    val turnInput = gamepad1.right_stick_x.toDouble()

                    drivebase.controlMotors(xInput, yInput, turnInput)

                    val liftInput = -gamepad2.left_stick_y.toDouble()
                    lift.setLiftPowerSafe(liftInput, gamepad2.b)

                    val extendInput = -gamepad2.right_stick_y.toDouble()

                    val inExtLimitUpper =
                        extender.extendPosition <= ExtenderConstants.MAX_EXTENSION_INCH
                    val inExtLimitLower = extender.extendPosition >= 0.7


                    extender.extendMotor.power =
                        if (extendInput > 0 && inExtLimitUpper || extendInput < 0 && inExtLimitLower) extendInput
                        else 0.0

                    val spinOut = gamepad2.right_bumper
                    val spinIn = gamepad2.right_trigger > 0.5

                    spintake.theSuckAction(spinIn, spinOut)

                    if (gamepad2.a && !pivotInputActive) spintake.switchWrist()
                    pivotInputActive = gamepad2.a
                }
            }


            while (opModeIsActive()) {
                drivebase.addTelemetry(telemetry)
                lift.addTelemetry(telemetry)
                extender.addTelemetry(telemetry)
                spintake.addTelemetry(telemetry)

                telemetry.status("running")

                yield()
            }

            driving.cancelAndJoin()

            drivebase.controlMotors(0.0, 0.0, 0.0)
            lift.setLiftPowerSafe(0.0, true)
            extender.extendMotor.power = 0.0
        }

    }
}