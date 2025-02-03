package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.systems.GoBildaPinpointDriver.DeviceStatus.CALIBRATING
import org.firstinspires.ftc.teamcode.utility.control.PoseData
import org.firstinspires.ftc.teamcode.utility.control.Vec2d
import org.firstinspires.ftc.teamcode.utility.rotate


class Odometry(
    hardwareMap: HardwareMap,
    val poseOffset: Pose2D? = null
) {
    private val odometry = hardwareMap.get(GoBildaPinpointDriver::class.java, "odometry")!!.apply {
        // x is sideways offset, positive left
        // y is front-back offset, positive forward
        // offsets in mm
        this.setOffsets(
            // thanks johnny
            119.75,
            -182.103,
        )

        this.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

        // x should increase when moving forward
        // y should increase when moving left
        this.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )
    }

    fun resetOdometry() {
        odometry.resetPosAndIMU()

        Thread.sleep(100)

        while (odometry.deviceStatus == CALIBRATING) {
            odometry.update()
        }
    }

    fun update() = odometry.update()

    val globalPose get() = PoseData(Vec2d(globalPosX, globalPosY), posRad)

    private val posXOffset get() = poseOffset?.getX(DistanceUnit.INCH) ?: 0.0
    private val posYOffset get() = poseOffset?.getY(DistanceUnit.INCH) ?: 0.0
    private val posRadOffset get() = poseOffset?.getHeading(AngleUnit.RADIANS) ?: 0.0

    val localPosX get() = odometry.position.getX(DistanceUnit.INCH)
    val localPosY get() = odometry.position.getY(DistanceUnit.INCH)

    private val globalPos get() = rotate(localPosX to localPosY, posRadOffset)
    val globalPosX get() = globalPos.first + posXOffset
    val globalPosY get() = globalPos.second + posYOffset

    private val localVels
        get() = rotate(
            globalVelX to globalVelY,
            -odometry.position.getHeading(AngleUnit.RADIANS)
        )
    val localVelX get() = localVels.first
    val localVelY get() = localVels.second

    val globalVelX get() = odometry.velX * 0.03937008
    val globalVelY get() = odometry.velY * 0.03937008

    val posRad get() = odometry.position.getHeading(AngleUnit.RADIANS) + posRadOffset
    val velRad get() = odometry.headingVelocity

    /*
    Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
    READY: the device is working as normal
    CALIBRATING: the device is calibrating and outputs are put on hold
    NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
    FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
    FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
    FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
    */
    private val deviceStatus get() = odometry.deviceStatus

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("robot pos x", globalPosX)
        telemetry.addData("robot pos y", globalPosY)
        telemetry.addData("robot heading", posRad)
        telemetry.addData("odometry status", deviceStatus)
        telemetry.addData("odometry frequency", odometry.frequency)
    }
}