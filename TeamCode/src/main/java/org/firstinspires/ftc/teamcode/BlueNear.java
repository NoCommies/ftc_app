package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by 4924_Users on 11/4/2017.
 */

@Disabled
@Autonomous(name = "BlueNear")
public class BlueNear extends Robot {

    public void init() {

        super.init();
        {

            CRYPTOBOX_CENTER_DISTANCE = 35.5;
        }

        {
            if (STARTING_POSITION.isNear()) {

                if (STARTING_POSITION.isBlue()) { //cryptobox is reversed when colors change

                    CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
                    CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
                } else {

                    CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
                    CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
                }
            } else { //STARTING_POSITION.isFar()

                CRYPTOBOX_LEFT_DISTANCE = -1;
                CRYPTOBOX_RIGHT_DISTANCE = -1;
            }
        }
    }

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    boolean isFinished = false;
    public void loop() {

        if(!isFinished) {

            driveWithEncoders(DRIVE_POWER, calculateInches());
            if(startingPosition().isRed()) reverseDriveBase();
            turnWithEncoders(TURN_POWER, -18);
            driveWithEncoders(DRIVE_POWER, 5);
            elapsedTime.reset();
            while(elapsedTime.time() < 3) collectionMotor.setPower(-0.5);
            collectionMotor.setPower(0);
            driveWithEncoders(DRIVE_POWER/2, 10);
            driveWithEncoders(DRIVE_POWER, -6);
            isFinished = true;
        }
    }
}
