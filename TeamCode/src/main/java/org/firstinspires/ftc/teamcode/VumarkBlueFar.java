package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/26/2017.
 */

@Autonomous(name = "VumarkBlueFar")
public class VumarkBlueFar extends Robot {

    boolean isFinished = false;

    {

        CRYPTOBOX_CENTER_DISTANCE = 16;
    }

    //initializing CRYPTOBOX_LEFT and RIGHT DISTANCE
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

    double calculateInches() {

        return CRYPTOBOX_RIGHT_DISTANCE;
        /*if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
        else return CRYPTOBOX_CENTER_DISTANCE;*/
    }

    @Override
    public void loop() {

        super.loop();
        if (!isFinished) {

            driveWithEncoders(DRIVE_POWER, 29);
            turnToPosition(TURN_POWER, 90);
            driveWithEncoders(DRIVE_POWER, calculateInches());
            turnToPosition(TURN_POWER, 0);
            isFinished = true;
        }
    }

    RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
