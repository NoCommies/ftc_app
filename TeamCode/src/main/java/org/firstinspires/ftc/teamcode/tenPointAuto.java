package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by 4924_Users on 11/3/2017.
 */
@Disabled
@Autonomous(name = "tenPointAutoFar")
public class tenPointAuto extends Robot {

    boolean notFinished = true;
    boolean notFinallyFinished = true;

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        super.loop();
        if (notFinished) {
            driveWithEncoders(DRIVE_POWER, 30);
            notFinished = false;
        }
        while (elapsedTime.time() <= 3) {
            collectionMotor.setPower(-0.5);
        }
        collectionMotor.setPower(0);
        if (notFinallyFinished) {
            driveWithEncoders(DRIVE_POWER, 2);
            driveWithEncoders(DRIVE_POWER, -4);
            notFinallyFinished = false;
        }

    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
