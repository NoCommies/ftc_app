package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Autonomous(name = "EncodersWithVumarkBlue")
public class EncodersWithVumarkBlue extends Robot {

    boolean notFinished = true;

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {
        if (notFinished) {
            driveWithEncoders(DRIVE_POWER, calculateInches(vuMark));
            turnToPosition(TURN_POWER, 270);
            driveWithEncoders(DRIVE_POWER, 5);
        }
        notFinished = false;

    }

    @Override
    public boolean isAutonomous() {

        return true;
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_CLOSE;
    }
}
