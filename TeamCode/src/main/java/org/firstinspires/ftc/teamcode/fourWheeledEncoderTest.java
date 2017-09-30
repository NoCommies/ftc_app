package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Autonomous(name = "fourwheelencoder")
public class fourWheeledEncoderTest extends OpMode {

    final double COUNTS_PER_INCH = 1120 * (32 / 48D) * 1 / 4D * 1 / Math.PI;
    int FINAL_POSITION;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor[] motors = {

            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
    };

    Vumark vumark = new Vumark();

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : motors) {

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(FINAL_POSITION + motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void init_loop() {


    }

    @Override
    public void start() {

        FINAL_POSITION = (int) (COUNTS_PER_INCH * calculateInches(vumark.vuMark));
        frontLeftMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
    }

    public void loop() {
//        if(frontLeftMotor.getCurrentPosition() >= 1000) frontLeftMotor.setPower(0);
        telemetry.addData("Path1", "Drove to%7d", frontLeftMotor.getCurrentPosition());
        telemetry.update();
    }

    public double calculateInches(RelicRecoveryVuMark vuMark) {

        telemetry.addData("vumark status", vuMark.toString());
        if (vuMark == RelicRecoveryVuMark.LEFT) return 35.5 - 6.5;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return 35.5 + 6.5;
        else return 35.5; //CENTER or UNKNOWN
    }
}
