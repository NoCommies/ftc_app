package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 4924_Users on 10/28/2017.
 */

@Autonomous(name = "autoTest")
public class Test extends OpMode {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public ElapsedTime time = new ElapsedTime();

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        time.reset();

    }

    @Override
    public void loop() {


        if (time.time() > 2 && time.time() < 3) frontLeftMotor.setPower(1);
        if (time.time() > 3 && time.time() < 4) frontLeftMotor.setPower(0);
        if (time.time() > 4 && time.time() < 5) frontRightMotor.setPower(1);
        if (time.time() > 5 && time.time() < 6) frontRightMotor.setPower(0);
        if (time.time() > 6 && time.time() < 7) backLeftMotor.setPower(1);
        if (time.time() > 7 && time.time() < 8) backLeftMotor.setPower(0);
        if (time.time() > 8 && time.time() < 9) backRightMotor.setPower(1);
        if (time.time() > 9 && time.time() < 10) backRightMotor.setPower(0);
    }
}
