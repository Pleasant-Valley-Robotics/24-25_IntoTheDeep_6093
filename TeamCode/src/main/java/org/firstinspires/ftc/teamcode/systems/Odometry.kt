package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit


class Odometry(hardwareMap: HardwareMap) {
    private val odometry = hardwareMap.get(GoBildaPinpointDriver::class.java, "odometry")!!.apply {
        // x is sideways offset, positive left
        // y is front-back offset, positive forward
        // offsets in mm
        this.setOffsets(-95.0, -192.0)

        this.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

        // x should increase when moving forward
        // y should increase when moving left
        this.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", this.xOffset);
//        telemetry.addData("Y offset", this.yOffset);
//        telemetry.addData("Device Version Number:", this.deviceVersion);
//        telemetry.addData("Device Scalar", this.yawScalar);
//        telemetry.update();

        this.resetPosAndIMU()
    }

    fun update() = odometry.update()

    val posX get() = odometry.position.getX(DistanceUnit.INCH)
    val posY get() = odometry.position.getY(DistanceUnit.INCH)
    val headingRad get() = odometry.position.getHeading(AngleUnit.RADIANS)

    /*
    Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
    READY: the device is working as normal
    CALIBRATING: the device is calibrating and outputs are put on hold
    NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
    FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
    FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
    FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
    */
    val deviceStatus get() = odometry.deviceStatus!!

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("robot pos x", posX)
        telemetry.addData("robot pos y", posY)
        telemetry.addData("robot heading", headingRad)
        telemetry.addData("odometry status", deviceStatus)
        telemetry.addData("odometry frequency", odometry.frequency)
    }
}