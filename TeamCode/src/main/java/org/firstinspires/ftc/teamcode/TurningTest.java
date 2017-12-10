package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 12/7/2017.
 */

@Autonomous(name = "TurningTest")
public class TurningTest extends Robot{

    private boolean isFinished = false;
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    public void loop() {

        telemetry.addData( "frontLedtMotor:", frontLeftMotor.getCurrentPosition());
        telemetry.addData("frontRightMotor:", frontRightMotor.getCurrentPosition());
        telemetry.addData("backLeftMotor:", backLeftMotor.getCurrentPosition());
        telemetry.addData("backRightMotor:", backRightMotor.getCurrentPosition());
        telemetry.update();
        if (!isFinished) {
            turnWithEncoders(TURN_POWER, 16);
            turnWithEncoders(TURN_POWER, -16);
            turnWithEncoders(TURN_POWER, 17);
            turnWithEncoders(TURN_POWER, -17);
            turnWithEncoders(TURN_POWER, 18);
            turnWithEncoders(TURN_POWER, -18);
            turnWithEncoders(TURN_POWER, 19);
            turnWithEncoders(TURN_POWER, -19);
            isFinished = true;
        }

    }
}
