package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Autonomous(name = "EncodersWithVumarkBlue")
public class EncodersWithVumarkBlue extends Robot {

    @Override
    public void init() {

        super.init();
    }

    @Override
    public void init_loop() {

        super.init_loop();
    }

    @Override
    public void start() {

        super.start();
        driveWithEncoders(0.5, calculateInches(vuMark));
    }

    @Override
    public void loop() {

        telemetry.addData("frontLeftMotor targets", frontLeftMotor.getTargetPosition());
        telemetry.addData("frontLeftMotor position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("frontLeftMotor mode", frontLeftMotor.getMode());
        telemetry.update();
    }

    @Override
    public boolean isAutonomous() {

        return true;
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_CLOSE;
    }
}
