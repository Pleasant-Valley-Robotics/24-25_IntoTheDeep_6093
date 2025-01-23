package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.utility.BucketConstants.BUCKET_IN_POS
import org.firstinspires.ftc.teamcode.utility.BucketConstants.BUCKET_OUT_POS
import org.firstinspires.ftc.teamcode.utility.BucketConstants.BUCKET_TOUCH_POS

/** the thing on the lift that scores samples in the basket */
class Bucket(hardwareMap: HardwareMap) {
    private val bucketServo = hardwareMap.servo.get("Bucket")

    enum class BucketState {
        In,
        Out,
        Touch,
    }

    /**
     * moves the flipper according to `param`. maps 0 to
     * [BUCKET_IN_POS] and 1 to [BUCKET_OUT_POS].
     *
     * @param param input to scale. `[0, 1]`
     * @see BUCKET_IN_POS
     * @see BUCKET_OUT_POS
     */
    fun pivotParam(param: Double) {
        bucketServo.position = BUCKET_IN_POS * (1 - param) + BUCKET_OUT_POS * param
    }

    /**
     * moves the flipper out or in
     *
     * @param state what to do with the flipper
     */
    fun pivotState(state: BucketState) {
        bucketServo.position = when (state) {
            BucketState.In -> BUCKET_IN_POS
            BucketState.Out -> BUCKET_OUT_POS
            BucketState.Touch -> BUCKET_TOUCH_POS
        }
    }
}