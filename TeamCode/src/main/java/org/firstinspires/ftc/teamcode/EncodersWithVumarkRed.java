package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 4924_Users on 10/13/2017.
 */

@Autonomous(name = "VumarkRedNear")
public class EncodersWithVumarkRed extends Robot {

    boolean notFinished = true;
    @Override
    public void loop() {

        super.loop();
        if (notFinished) {
            driveWithEncoders(-DRIVE_POWER, calculateInches());

            setMotorsTargets(-22, new DcMotor[]{frontRightMotor, backRightMotor});
            //setMotorsTargets(840, new DcMotor[]{backRightMotor, frontRightMotor});

            setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, new DcMotor[]{frontRightMotor, backRightMotor});

            setMotorsPowers(TURN_POWER, new DcMotor[]{frontRightMotor, backRightMotor});
            while ((frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) < 0) {

                setMotorsPowers(-TURN_POWER, new DcMotor[]{frontRightMotor, backRightMotor});
            }
            while ((frontRightMotor.getCurrentPosition() - frontRightMotor.getTargetPosition()) > 10) {
                setMotorsPowers(TURN_POWER, new DcMotor[]{frontRightMotor, backRightMotor});
            }
            setMotorsPowers(0, DRIVE_BASE_MOTORS);
        }
        notFinished = false;
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
