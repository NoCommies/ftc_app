package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 11/4/2017.
 */

@Autonomous(name = "BlueFar")
public class BlueFar extends Robot {

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    public void init() {

        super.init();
        CRYPTOBOX_CENTER_DISTANCE = 11;
        if (STARTING_POSITION.isBlue()) { //cryptobox is reversed when colors change

            CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
            CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
        } else {

            CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
            CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
        }
    }

    boolean isFinished = false;


    public void loop() {

        super.loop();
        if(!isFinished) {

            driveWithEncoders(DRIVE_POWER, 26);
            turnWithEncoders(TURN_POWER, 18);
            driveWithEncoders(DRIVE_POWER, calculateInches());
            if(startingPosition().isRed()) reverseDriveBase();
            turnWithEncoders(TURN_POWER, -18.5);
            elapsedTime.reset();
            while(elapsedTime.time() < 3) collectionMotor.setPower(-0.5);
            collectionMotor.setPower(0);
            driveWithEncoders(DRIVE_POWER/2, 10);
            driveWithEncoders(DRIVE_POWER, -6);
            isFinished = true;
        }
    }
}
