/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

*/
/**
 * Created by 4924_Users on 10/11/2017.
 * <p>
 * Created by 4924_Users on 10/11/2017.
 * <p>
 * Created by 4924_Users on 10/11/2017.
 *//*


@Autonomous(name = "BlueCloseMoveAndTurn")
*/
/**
 * Created by 4924_Users on 10/11/2017.
 *//*


public class AutonomousTest extends Robot {

    public void init() {

        telemetry.addData("Starting super.init()...", "");
        super.init();
        telemetry.addData("Setting encoder targets...", "");
        telemetry.addData("frontLeftMotor encoders", frontLeftMotor.getCurrentPosition());
        ENCODER_TARGET_POSITION = (int) (10*COUNTS_PER_INCH);
        frontLeftMotor.setTargetPosition(ENCODER_TARGET_POSITION+frontLeftMotor.getCurrentPosition());
        frontRightMotor.setTargetPosition(ENCODER_TARGET_POSITION+frontRightMotor.getCurrentPosition());
        backLeftMotor.setTargetPosition(ENCODER_TARGET_POSITION+backLeftMotor.getCurrentPosition());
        backRightMotor.setTargetPosition(ENCODER_TARGET_POSITION+backRightMotor.getCurrentPosition());
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void start() {

        super.start();
        telemetry.addData("Giving motors power...", "");
        frontLeftMotor.setPower(0.25);
        frontRightMotor.setPower(0.25);
        backLeftMotor.setPower(0.25);
        backRightMotor.setPower(0.25);
    }

    public void loop() {



        while(elapsedTime.time() > 10 && elapsedTime.time() < 15) {

            telemetry.addData("elapsed time", elapsedTime.time());

            telemetry.addData("Turning CCW...", "");

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLeftMotor.setPower(0.25);
            frontRightMotor.setPower(-0.25);
            backLeftMotor.setPower(0.25);
            backRightMotor.setPower(-0.25);

        }

        while(elapsedTime.time() > 15 && elapsedTime.time() < 17) {

            telemetry.update();

            telemetry.addData("Holonomic...", "");

            frontLeftMotor.setPower(0.25);
            frontRightMotor.setPower(-0.25);
            backLeftMotor.setPower(-0.25);
            backRightMotor.setPower(0.25);
        }
    }

    public boolean isAutonomous() {

        return true;
    }

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_CLOSE;
    }
}
*/
