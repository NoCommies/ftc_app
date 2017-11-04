package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 4924_Users on 10/14/2017.
 */

@Autonomous(name = "GyroTurningTest")
public class TurnGyroTest extends Robot {

    boolean finished = false;

    @Override
    public void start() {

        super.start();
        setMotorsModes(DcMotor.RunMode.RUN_USING_ENCODER, DRIVE_BASE_MOTORS);
        //turnUsingOffset(179);
    }

    @Override
    public void loop() {

        super.loop();
        telemetry.addData("Heading", getHeading());
        telemetry.addData("Offset", offset);
        telemetry.addData("H+O", getHeading() - offset);
        telemetry.update();
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
