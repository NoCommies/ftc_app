package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Autonomous(name = "EncodersWithVumark")
public class EncodersWithVumark extends Robot {

    @Override
    public void init() {
        super.init();
        telemetry.addData("Init has started", "init()"); //LEFT, CENTER, or RIGHT, useful for debugging
        telemetry.update();
        telemetry.addData("Motors have been initialized", "motors"); //LEFT, CENTER, or RIGHT, useful for debugging
        telemetry.update();
    }

    @Override
    public void init_loop() {

        vuMark = RelicRecoveryVuMark.from(relicTemplate); //LEFT, CENTER, or RIGHT if read, UNKNOWN if undetermined reading
        telemetry.addData("vuMark", vuMark.toString()); //LEFT, CENTER, or RIGHT, useful for debugging
        telemetry.update();
    }

    @Override
    public void start() {

        ENCODER_TARGET_POSITION = (int) (COUNTS_PER_INCH * calculateInches(vuMark));
        //desired encoder targets
        frontLeftMotor.setTargetPosition(ENCODER_TARGET_POSITION + frontLeftMotor.getCurrentPosition());
        frontRightMotor.setTargetPosition(ENCODER_TARGET_POSITION + frontRightMotor.getCurrentPosition());
        backLeftMotor.setTargetPosition(ENCODER_TARGET_POSITION + backLeftMotor.getCurrentPosition());
        backRightMotor.setTargetPosition(ENCODER_TARGET_POSITION + backRightMotor.getCurrentPosition());

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);

    }

    public void loop() {

    }

    public double calculateInches(RelicRecoveryVuMark vuMark) {

        telemetry.addData("Starting calculateInches...", "");
        try {
            if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
            else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
            else if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) //UNKNOWN is grouped with CENTER because CENTER is easiest to place
                return CRYPTOBOX_CENTER_DISTANCE;
        } catch (NullPointerException e) {

            return CRYPTOBOX_CENTER_DISTANCE;
        }

        telemetry.addData("vumark not read", "");
        return calculateInches(RelicRecoveryVuMark.CENTER); //never reached because all enum values are covered, but necessary for method to compile
    }

    boolean requireAccelerometer() {

        return false;
    }

    boolean requireGyro() {

        return false;
    }

    boolean requireVuforia() {

        return true;
    }

    boolean isRed() {

        return true;
    }
}
