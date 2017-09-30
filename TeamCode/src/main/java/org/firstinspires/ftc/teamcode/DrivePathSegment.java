package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/18/2015.
 */
public class DrivePathSegment {

    public static final int LINEAR = 0;
    public static final int TURN = 1;
    public static final int HOLONOMIC = 2;
    public static final int STOP = 0;

    public float LeftSideDistance = 0.0f;
    public float RightSideDistance = 0.0f;
    public float Angle = 0.0f;
    public float leftPower = 0.0f;
    public float rightPower = 0.0f;
    public float delayTime = 0.0f;
    public boolean isTurn = false;
    public boolean isHolonomic = false;
    public boolean isDelay = false;
    public boolean isClockwise = false;

    public DrivePathSegment() {
    }

    public DrivePathSegment(float moveTarget, float power, int type) {

        if (type == LINEAR) {

            LeftSideDistance = moveTarget;
            RightSideDistance = moveTarget;
            leftPower = power;
            rightPower = power;
            isTurn = false;
            isHolonomic = false;
            isDelay = false;
        }

        if (type == TURN) {

            Angle = moveTarget;
            leftPower = power;
            rightPower = power;
            isTurn = true;
            isHolonomic = false;
            isDelay = false;
        }

        if (type == HOLONOMIC) {

            LeftSideDistance = moveTarget;
            RightSideDistance = moveTarget;
            leftPower = -power;
            rightPower = -power;
            isTurn = false;
            isHolonomic = true;
            isDelay = false;
        }
    }

    public DrivePathSegment(float timeDelay) {

        delayTime = timeDelay;
        isDelay = true;
    }
}
