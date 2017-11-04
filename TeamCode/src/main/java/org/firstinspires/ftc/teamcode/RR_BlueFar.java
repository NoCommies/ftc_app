package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 4924_Users on 10/26/2017.
 */

@Autonomous(name = "RR_BlueFar")
public class RR_BlueFar extends Robot {

    boolean isFinished = false;

    public void init() {

        super.init();
        CRYPTOBOX_CENTER_DISTANCE = 16;
        if (STARTING_POSITION.isBlue()) { //cryptobox is reversed when colors change

            CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
            CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
        } else {

            CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
            CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
        }

    }

    @Override
    public void loop() {

        super.loop();
        if (!isFinished) {
            /*driveWithEncoders(DRIVE_POWER, 29);
            turnToPosition(TURN_POWER, 90);
            driveWithEncoders(DRIVE_POWER, calculateInches());
            turnToPosition(TURN_POWER, 0);
            isFinished = true;*/
            /*
            driveWithEncoders(DRIVE_POWER, 15);
            turnToPosition(TURN_POWER, 90);
            driveWithEncoders(DRIVE_POWER, calculateInches());
            turnToPosition(TURN_POWER, 0);
            driveWithEncoders(DRIVE_POWER, 5);
            collectionMotor.setPower(-1);*/
            setMotorsTargets(22, new DcMotor[]{frontLeftMotor, backLeftMotor});
            //setMotorsTargets(840, new DcMotor[]{backRightMotor, frontRightMotor});

            setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, new DcMotor[]{frontLeftMotor, backLeftMotor});

            setMotorsPowers(TURN_POWER, new DcMotor[]{frontLeftMotor, backLeftMotor});
            while ((frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < 0) {

                setMotorsPowers(-TURN_POWER, new DcMotor[]{frontLeftMotor, backLeftMotor});
                setMotorsPowers(0.3, new DcMotor[]{frontRightMotor, backRightMotor});
            }
            while ((frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) > 10) {
                setMotorsPowers(TURN_POWER, new DcMotor[]{frontLeftMotor, backLeftMotor});
                setMotorsPowers(-0.3, new DcMotor[]{frontRightMotor, backRightMotor});
            }
            setMotorsPowers(0, DRIVE_BASE_MOTORS);
            driveWithEncoders(DRIVE_POWER, calculateInches());
            setMotorsTargets(0, new DcMotor[]{frontLeftMotor, backLeftMotor});
            //setMotorsTargets(840, new DcMotor[]{backRightMotor, frontRightMotor});

            setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, new DcMotor[]{frontLeftMotor, backLeftMotor});

            setMotorsPowers(TURN_POWER, new DcMotor[]{frontLeftMotor, backLeftMotor});
            while ((frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) < 0) {

                setMotorsPowers(-TURN_POWER, new DcMotor[]{frontLeftMotor, backLeftMotor});
            }
            while ((frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) > 10) {
                setMotorsPowers(TURN_POWER, new DcMotor[]{frontLeftMotor, backLeftMotor});
            }
            setMotorsPowers(0, DRIVE_BASE_MOTORS);
            isFinished = true;


        }
    }

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
