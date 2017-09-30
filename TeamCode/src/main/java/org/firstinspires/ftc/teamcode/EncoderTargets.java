package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/18/2015.
 */
public class EncoderTargets {

    public int frontRightTarget;
    public int frontLeftTarget;
    public int backRightTarget;
    public int backLeftTarget;

    public EncoderTargets(int left, int right) {

        frontLeftTarget = left;
        frontRightTarget = right;
        backLeftTarget = left;
        backRightTarget = right;
    }
}
