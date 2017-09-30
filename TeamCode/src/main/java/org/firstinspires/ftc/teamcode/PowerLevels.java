package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/8/2016.
 */

public class PowerLevels {

    float frontRightPower = 0.0f;
    float frontLeftPower = 0.0f;
    float backRightPower = 0.0f;
    float backLeftPower = 0.0f;

    public PowerLevels(float frontLeft, float frontRight, float backLeft, float backRight) {

        this.frontRightPower = frontRight;
        this.frontLeftPower = frontLeft;
        this.backRightPower = backRight;
        this.backLeftPower = backLeft;
    }
}
