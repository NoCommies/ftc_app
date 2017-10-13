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
    private static final double CRYPTOBOX_OFFSET = 6.5; //offset of left/right areas of cryptobox
    private static final int ENCODER_TICKS_PER_ROTATION = 1120; //encoder counts per shaft turn
    private static final double GEAR_RATIO = 32 / 48D; //48 teeth on motor gear, 32 teeth on wheel gear
    private static final double WHEEL_DIAMETER = 4;

    /* ENCODER SPECS*/
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //wheel diameter * pi
    static final double COUNTS_PER_INCH =
            (ENCODER_TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    /* MOTORS */
    static DcMotor frontLeftMotor;
    static DcMotor frontRightMotor;
    static DcMotor backLeftMotor;
    //encoder ticks per inch moved
    //static int ENCODER_TARGET_POSITION;

    /* HARDWARE */

    //declares our hardware, initialized later in init()
    static DcMotor backRightMotor;
    static DcMotor[] DRIVE_BASE_MOTORS = new DcMotor[4];
    static DcMotor[] ALL_MOTORS;
    /* VUFORIA PARAMETERS */
    //fields for camera recognition
    static RelicRecoveryVuMark vuMark; //enum set based on pictogram reading
    static VuforiaLocalizer vuforia; //later initialized with (sic) parameters
    static VuforiaTrackables relicTrackables;

        /* SENSORS */
        static VuforiaTrackable relicTemplate;
    // from cryptobox center in inches, should be 6.5, but exaggerated for testing
    final double CRYPTOBOX_LEFT_DISTANCE;
    final double CRYPTOBOX_RIGHT_DISTANCE;
    /* TIME */
    ElapsedTime elapsedTime = new ElapsedTime();
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    {
        if (startingPosition().isClose()) {

            if (startingPosition().isBlue()) { //cryptobox is reversed when colors change

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

    public static void driveWithEncoders(double movePower, double moveDistanceInInches) {

        setDriveBaseTargets(moveDistanceInInches);

        setDriveBaseMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDriveBasePowers(movePower);
    }
    //parameters later used to initialize vuforia

    public static void setDriveBaseTargets(double encoderTargetInInches) {

        setMotorsTargets(encoderTargetInInches, DRIVE_BASE_MOTORS);
    }

    public static void setDriveBasePowers(double power) {

        setMotorsPowers(power, DRIVE_BASE_MOTORS);
    }

    public static void setDriveBaseMode(DcMotor.RunMode runMode) {

        setMotorsModes(runMode, DRIVE_BASE_MOTORS);
    }

    public static void setMotorsTargets(double encoderTargetInInches, DcMotor[] motors) {

        for (DcMotor d : motors) {
            d.setTargetPosition(
                    (int) (COUNTS_PER_INCH * encoderTargetInInches) + d.getCurrentPosition());
        }

        /*DRIVE_BASE_MOTORS[0].setTargetPosition((int) (COUNTS_PER_INCH*encoderTargetInInches) + DRIVE_BASE_MOTORS[0].getCurrentPosition());
        DRIVE_BASE_MOTORS[1].setTargetPosition((int) (COUNTS_PER_INCH*encoderTargetInInches) + DRIVE_BASE_MOTORS[1].getCurrentPosition());
        DRIVE_BASE_MOTORS[2].setTargetPosition((int) (COUNTS_PER_INCH*encoderTargetInInches) + DRIVE_BASE_MOTORS[2].getCurrentPosition());
        DRIVE_BASE_MOTORS[3].setTargetPosition((int) (COUNTS_PER_INCH*encoderTargetInInches) + DRIVE_BASE_MOTORS[3].getCurrentPosition());
*/
    }

    public static void setMotorsPowers(double power, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setPower(power);
    }

    public static void setMotorsModes(DcMotor.RunMode runMode, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setMode(runMode);
    }

    public static void turnWithGyro(double turnPower, double turnAngleOffset) {
        //turnAngleOffset expressed in clockwise-degrees off of the current heading

    }

    public static void turnToPosition(double turnPower, double desiredHeading) {
        //desiredHeading is the angle measure, in degrees, that we want to to

    }

    abstract RobotPosition startingPosition();

    //true for Autonomous, false for TeleOp
    abstract boolean isAutonomous();

    public double calculateInches(RelicRecoveryVuMark vuMark) {

        telemetry.addData("Starting calculateInches...", "");
        try {

            if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
            else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
            else if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) //UNKNOWN is grouped with CENTER because CENTER is easiest to place
                return CRYPTOBOX_CENTER_DISTANCE;

            else throw new NullPointerException("vuMark is outside of expected range");

        } catch (NullPointerException e) {

            return CRYPTOBOX_CENTER_DISTANCE;
        }
    }

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

        DRIVE_BASE_MOTORS[0] = frontLeftMotor;
        DRIVE_BASE_MOTORS[1] = frontRightMotor;
        DRIVE_BASE_MOTORS[2] = backLeftMotor;
        DRIVE_BASE_MOTORS[3] = backRightMotor;

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

    public void init_loop() {

        if (isAutonomous()) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate); //LEFT, CENTER, or RIGHT if read, UNKNOWN if undetermined reading
            telemetry.addData("vuMark", vuMark.toString()); //LEFT, CENTER, or RIGHT, useful for debugging
            telemetry.update();
        }
    }

    public void start() {

        elapsedTime.reset(); //since init() takes so long, elapsedTime is delayed, so we reset
    }
}


