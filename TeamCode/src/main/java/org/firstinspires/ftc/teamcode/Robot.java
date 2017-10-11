package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by 4924_Users on 10/7/2017.
 */

public abstract class Robot extends OpMode {

    /* FIELD PARAMETERS */

    //parameters specific to our field position and orientation
    static final double CRYPTOBOX_CENTER_DISTANCE = 35.5; //distance from center of relic-side blue
    // balance board to center of associated cryptobox center this will have to change as we work on
    // the other starting positions
    private static final double CRYPTOBOX_OFFSET = 15; //offset of left/right areas of cryptobox from
    /* ENCODER SPECS*/
    private static final int ENCODER_TICKS_PER_ROTATION = 1120; //encoder counts per shaft turn
    private static final double GEAR_RATIO = 32 / 48D; //48 teeth on motor gear, 32 teeth on wheel gear
    private static final double WHEEL_DIAMETER = 4;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //wheel diameter * pi
    static final double COUNTS_PER_INCH =
            (ENCODER_TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;

    /* HARDWARE */

    //declares our hardware, initialized later in init()
    static int ENCODER_TARGET_POSITION;
    /* VUFORIA PARAMETERS */
    //fields for camera recognition
    static RelicRecoveryVuMark vuMark; //enum set based on pictogram reading
    static VuforiaLocalizer vuforia; //later initialized with (sic) parameters
    static VuforiaTrackables relicTrackables;
    static VuforiaTrackable relicTemplate;

        /* SENSORS */

    /* MECHANICAL PARAMETERS */

    //specs of the hardware we are using. ex: ENCODER_TICKS_PER_ROTATION
    //cryptobox center in inches
    //actual value should be 6.5, offset is exaggerated for testing purposes
    final double CRYPTOBOX_LEFT_DISTANCE;
    final double CRYPTOBOX_RIGHT_DISTANCE;
    /* MOTORS */
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    //encoder ticks per inch moved
    /* TIME */
    ElapsedTime elapsedTime = new ElapsedTime();
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    {
        if (startingPosition().isClose()) {

            if (startingPosition().isBlue()) {

                CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
                CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
            } else {

                CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
                CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
            }
        } else { //startingPosition.isFar()

            CRYPTOBOX_LEFT_DISTANCE = -1;
            CRYPTOBOX_RIGHT_DISTANCE = -1;
        }
    }

    public static void reverse(DcMotor d) { //reverses the set direction of the motor

        if (d.getDirection() == DcMotorSimple.Direction.FORWARD)
            d.setDirection(DcMotorSimple.Direction.REVERSE);
        else if (d.getDirection() == DcMotorSimple.Direction.REVERSE)
            d.setDirection(DcMotorSimple.Direction.FORWARD);
        else throw new IllegalArgumentException("Motor type is not set, cannot be reversed");
    }

    abstract RobotPosition startingPosition();

    //true for Autonomous, false for TeleOp
    abstract boolean isAutonomous();
    //parameters later used to initialize vuforia

    public void init() {

        //these names are set in the configuration on the Robot Controller phone
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //one set of motors has to be reversed because they are facing a different way
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //on the red board, the robot will be facing backwards, so we want to reverse the motors
        if (startingPosition().isRed()) {
            reverse(frontLeftMotor);
            reverse(frontRightMotor);
            reverse(backLeftMotor);
            reverse(backRightMotor);
        }

        //we want the motors' encoders to be set to 0 when robot is initialized
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (isAutonomous()) { //below is Autonomous-only code, this removes unnecessary loading

            parameters.vuforiaLicenseKey = "AdDqKyD/////AAAAGQ/rpKTVlUiMmdwxDFRT5LiD8kI3QucN9xL8BbQRw/rYsleEjKBm/GOQB4GnSmvyzTFNFOBfZQ9o06uen5gYZvJthDx8OSVm78QegaFqHEGPjDRtqIAuLxz+12HVQXbIutqXfR595SNIl0yKUbbXFTq21ElXEDDNwO0Lv8ptnJPLib85+omkc5c8xfG6oNIhFg+sPIfCrpFACHdrr23MpY8AzLHiYleHnhpyY/y/IqsXw7CYPV2kKY70GEwH8I0MGxBw8tw8EoYpXk4vxUzHAPfgvBDztFz3x9fpcxoeqb0jl2L7GB7Aq7u+Sea+g4FoTG/9FD4rEy4I/Lz+OjdbE2eEUCGnyy10Q5o3AGG5R3cW";
            //license key set, necessary for vuforia code to be used
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //uses front camera of Robot Controller for detection
            //if above code is changed to ...CameraDirection.BACK;, the back-facing camera will be used instead

            /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);*/
            //if above code is enabled, the Driver Station will display the camera reading from the Robot Controller on its screen

            vuforia = ClassFactory.createVuforiaLocalizer(parameters); //vuforia object initialized based on set parameters
            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTrackables.activate(); //start listening for camera's data
        }
    }

    public void start() {

        elapsedTime.reset();
    }
}


