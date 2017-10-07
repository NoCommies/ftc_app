package ExperimentalClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 4924_Users on 9/30/2017.
 */

@TeleOp

public class Ayca_Hello_World extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {
  /*      imu = hardwareMap.get(Gyroscope.class, "imu");
       motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        servoTest = hardwareMap.get(Servo.class, "servoTest"); */

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int t = 0;
        double tgtPower = 0;
        waitForStart();

        while (opModeIsActive()) {
            /*t++;
            telemetry.addData("Number of Times Ran = " + t, "Running");
            telemetry.addData("Always", "Save");
            telemetry.update();*/
            tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();


        }
    }


}

