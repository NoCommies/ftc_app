package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/28/2017.
 */
@Autonomous(name = "NearVuMarkBlueRR")

public class VumarkBlueNearNewRobot extends RR_Robot {

    boolean notFinished = true;

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

    double calculateInches() {

        return CRYPTOBOX_RIGHT_DISTANCE;
        /*if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
        else return CRYPTOBOX_CENTER_DISTANCE;*/
    }

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        super.loop();
        if (notFinished) {
            /*
            driveWithEncoders(DRIVE_POWER, 5);
            turnToPosition(TURN_POWER, 90);
            driveWithEncoders(DRIVE_POWER, 5);*/
            turnWithEncoders(TURN_POWER, 280, 840);
            driveWithEncoders(DRIVE_POWER, 20);
            turnWithEncoders(TURN_POWER, 560, 560);
        }
        notFinished = false;
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
