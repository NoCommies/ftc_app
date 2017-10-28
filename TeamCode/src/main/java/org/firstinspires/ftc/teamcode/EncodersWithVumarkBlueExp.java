package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 4924_Users on 10/28/2017.
 */

@Autonomous(name = "EncodersWithVumarkBlueExp")
public class EncodersWithVumarkBlueExp extends Robot {

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


    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        super.loop();
        if (notFinished) {
            driveWithEncoders(DRIVE_POWER, calculateInches());

            setMotorsTargets(-22, new DcMotor[]{frontLeftMotor, backLeftMotor});
            setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, new DcMotor[]{frontLeftMotor, backLeftMotor});
            turn(TURN_POWER, RotationalDirection.COUNTER_CLOCKWISE);
            setMotorsPowers(0, DRIVE_BASE_MOTORS);
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
