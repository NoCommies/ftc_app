package ExperimentalClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Disabled
@Autonomous(name = "TurnOneRotation")
public class encoderOneRotation extends OpMode {

    DcMotor frontLeftMotor;

    @Override
    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setTargetPosition(1120 + frontLeftMotor.getCurrentPosition());
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void start() {
        frontLeftMotor.setPower(0.5);
    }

    public void loop() {
//        if(frontLeftMotor.getCurrentPosition() >= 1000) frontLeftMotor.setPower(0);
        telemetry.addData("Path1", "Drove to%7d", frontLeftMotor.getCurrentPosition());
        telemetry.update();
    }
}
