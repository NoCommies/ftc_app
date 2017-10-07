package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Autonomous(name = "EncodersWithVumark")
public class EncodersWithVumark extends OpMode {

    final int ENCODER_TICKS_PER_ROTATION = 1120; //how many times the encoder counts every rotation of the motor shaft
    final double GEAR_RATIO = 32 / 48D; //48 teeth on motor gear, 32 teeth on wheel gear
    final double WHEEL_CIRCUMFERENCE = 4 * Math.PI; //wheel diameter * pi
    final double COUNTS_PER_INCH = (ENCODER_TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE; //number of encoder ticks per inch moved
    final double CRYPTOBOX_CENTER_DISTANCE = 35.5; //distance from center of relic-side blue balance board to center of associated cryptobox center this will have to change as we work on the other starting positions
    final double CRYPTOBOX_OFFSET = 15; //offset of left/right areas of cryptobox from cryptobox center in inches
    //actual value should be 6.5, offset is exaggerated for testing purposes
    final double CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET; //moves robot less to reach left (close) side of cryptobox
    final double CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET; //moves robot more to reach right (far) side of cryptobox
    public RelicRecoveryVuMark vuMark; //enum set based on pictogram reading
    int ENCODER_TARGET_POSITION; //distance we want to go
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    VuforiaLocalizer vuforia; //later initialized with (sic) parameters
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    @Override
    public void init() {
        telemetry.addData("Init has started", "init()"); //LEFT, CENTER, or RIGHT, useful for debugging
        telemetry.update();
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor"); //initialization based on phone configuration
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //must be reversed because motor is facing a different direction
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); //must be reversed because motor is facing a different direction
        telemetry.addData("Motors have been initialized", "motors"); //LEFT, CENTER, or RIGHT, useful for debugging
        telemetry.update();
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);*/
        //if above code is enabled, the Driver Station will display the camera reading from the Robot Controller on its screen

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //parameters later used to initialize vuforia
        parameters.vuforiaLicenseKey = "AdDqKyD/////AAAAGQ/rpKTVlUiMmdwxDFRT5LiD8kI3QucN9xL8BbQRw/rYsleEjKBm/GOQB4GnSmvyzTFNFOBfZQ9o06uen5gYZvJthDx8OSVm78QegaFqHEGPjDRtqIAuLxz+12HVQXbIutqXfR595SNIl0yKUbbXFTq21ElXEDDNwO0Lv8ptnJPLib85+omkc5c8xfG6oNIhFg+sPIfCrpFACHdrr23MpY8AzLHiYleHnhpyY/y/IqsXw7CYPV2kKY70GEwH8I0MGxBw8tw8EoYpXk4vxUzHAPfgvBDztFz3x9fpcxoeqb0jl2L7GB7Aq7u+Sea+g4FoTG/9FD4rEy4I/Lz+OjdbE2eEUCGnyy10Q5o3AGG5R3cW";
        //license key set, necessary for vuforia code to be used
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //uses front camera of Robot Controller for detection
        //if above code is changed to ...CameraDirection.BACK;, the back-facing camera will be used instead
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //vuforia object initialized based on set parameters
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate(); //start listening for camera's data
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
        //ENCODER_TARGET_POSITION = (int) (COUNTS_PER_INCH * 30);
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

        if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
        else if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN)
            return CRYPTOBOX_CENTER_DISTANCE;
        //UNKNOWN is grouped with CENTER because CENTER is easiest to place

        throw new IllegalArgumentException(); //never reached because all enum values are covered, but necessary for method to compile
    }
}
