package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by user6 on 11/11/2017.
 */

public class IterativeRobotTenPoints extends LinearOpMode {

    /* FIELD PARAMETERS *///commit_file

    protected static final double DRIVE_POWER = 0.4;
    protected static final double TURN_POWER = 0.3;
    //to center of cryptobox
    protected static final double CRYPTOBOX_OFFSET = 6.5; //offset of left/right areas of cryptobox
    /* ROBOT CONSTANTS*/
    private static final int ENCODER_TICKS_PER_ROTATION = 1120; //encoder counts per shaft turn
    private static final int MOTOR_TEETH = 32;
    private static final int WHEEL_TEETH = 16;
    private static final double GEAR_RATIO = WHEEL_TEETH / (double) MOTOR_TEETH; //48 teeth on motor gear, 32 teeth on wheel gear
    private static final double WHEEL_DIAMETER = 4;
    /* HARDWARE */
    //declares our hardware, initialized later in init()
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //wheel diameter * pi
    private static final double COUNTS_PER_INCH =
            (ENCODER_TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    private static final double GYRO_TURN_TOLERANCE_DEGREES = 5;
    private static final int ENCODER_TOLERANCE = 10;
    protected static Telemetry staticTelemetry;
    protected static double CRYPTOBOX_CENTER_DISTANCE = Double.MIN_VALUE; //distance from center of relic-side
    protected static RobotPosition STARTING_POSITION;
    protected static DcMotor[] DRIVE_BASE_MOTORS = new DcMotor[4];
    protected static DcMotor[] ALL_MOTORS = new DcMotor[4];
    /* TIME */
    protected static ElapsedTime elapsedTime = new ElapsedTime();
    static double offset = 0;
    /* SENSORS */
    static BNO055IMU imu;
    /* NAVIGATION */
    static BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
    /* VUFORIA */
    //fields for camera recognition
    static RelicRecoveryVuMark vuMark; //enum set based on pictogram reading
    /* STATES *//*

    enum STATE {

        DRIVE,
        TURN,
        STOP;

        STATE[] toArray() {


        }
    }*/
    // from cryptobox center in inches, should be 6.5, but exaggerated for testing
    static double CRYPTOBOX_LEFT_DISTANCE;
    static double CRYPTOBOX_RIGHT_DISTANCE;
    /* MOTORS */
    static DcMotor frontLeftMotor;
    static DcMotor frontRightMotor;
    static DcMotor backLeftMotor;
    //encoder ticks per inch moved
    static DcMotor backRightMotor;
    private static Acceleration acceleration;
    private static VuforiaLocalizer vuforia; //later initialized with (sic) vuforiaParameters
    private static VuforiaTrackables relicTrackables;
    private static VuforiaTrackable relicTemplate;
    DcMotor collectionMotor;
    private VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
    private DcMotor relicExtension;
    private DcMotor deliveryMotor;
    private Servo barServo;
    private CRServo elbowServo;

    {

        STARTING_POSITION = startingPosition();
    }

    {
        msStuckDetectInit = 10000;
        msStuckDetectLoop = 15000;
    }

    static void reverse(DcMotor d) {

        d.setDirection(d.getDirection().inverted());
    }

    protected static void reverseDriveBase() {

        reverse(frontLeftMotor);
        reverse(frontRightMotor);
        reverse(backLeftMotor);
        reverse(backRightMotor);
    }

    protected static void driveWithEncoders(double movePower, double moveDistanceInInches) {

        setMotorsTargets(moveDistanceInInches, DRIVE_BASE_MOTORS);

        setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, DRIVE_BASE_MOTORS);

        while (Math.abs(DRIVE_BASE_MOTORS[0].getCurrentPosition() - DRIVE_BASE_MOTORS[0].getTargetPosition()) > ENCODER_TOLERANCE)

            setMotorsPowers(movePower, DRIVE_BASE_MOTORS);

        setMotorsPowers(0, DRIVE_BASE_MOTORS);
    }

    protected static void setMotorsTargets(double encoderTargetInInches, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setTargetPosition(
                    (int) (COUNTS_PER_INCH * encoderTargetInInches) + d.getCurrentPosition());
    }


    protected static void setMotorsPowers(double power, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setPower(power);
    }

    protected static void setMotorsModes(DcMotor.RunMode runMode, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setMode(runMode);
    }

    protected static void turn(double power, RotationalDirection direction) {

        if (direction == RotationalDirection.CLOCKWISE) {

            frontLeftMotor.setPower(-power);
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
        } else if (direction == RotationalDirection.COUNTER_CLOCKWISE) {

            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
        } else
            throw new IllegalArgumentException("RotationalDirection may be clockwise or counter-clockwise only");
    }


    protected static void turnToPosition(double turnPower, double desiredHeading) {
        //desiredHeading is the angle that we want to move to, it should be -180<x<180
        setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DRIVE_BASE_MOTORS);
        formatAngle(desiredHeading);
        //we reduce desiredHeading just in case it is formatted incorrectly
        while (formatAngle((getHeading()) - desiredHeading) > GYRO_TURN_TOLERANCE_DEGREES) {

            writeTelemetry(getHeading());
            turn(turnPower, RotationalDirection.COUNTER_CLOCKWISE);
        }
        //if it is faster to turn CCW, then robot turns CCW

        while (formatAngle((getHeading()) - desiredHeading) < -GYRO_TURN_TOLERANCE_DEGREES) {

            writeTelemetry(getHeading());
            turn(turnPower, RotationalDirection.CLOCKWISE);
        }
        setMotorsPowers(0, DRIVE_BASE_MOTORS);
        //defaults to CW turning
    }

    protected static void turnWithEncoders(double turnPower, double desiredHeading) {

        setMotorsTargets(desiredHeading, new DcMotor[]{frontLeftMotor, backLeftMotor});
        setMotorsTargets(-desiredHeading, new DcMotor[]{frontRightMotor, backRightMotor});


        setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, DRIVE_BASE_MOTORS);
        //setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, new DcMotor[] {frontRightMotor, backLeftMotor, backRightMotor});
        while (Math.abs(DRIVE_BASE_MOTORS[0].getCurrentPosition() - DRIVE_BASE_MOTORS[0].getTargetPosition()) > ENCODER_TOLERANCE) {

            writeTelemetry(getHeading());
            frontLeftMotor.setPower(-turnPower);
            frontRightMotor.setPower(turnPower);
            backLeftMotor.setPower(-turnPower);
            backRightMotor.setPower(turnPower);
            writeTelemetry(DRIVE_BASE_MOTORS[0].getTargetPosition());
        }/*
        if ((frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition()) > 5) {
            turn(turnPower, RotationalDirection.COUNTER_CLOCKWISE);
        }*/
        setMotorsPowers(0, DRIVE_BASE_MOTORS);
    }

    static double formatAngle(double angle) {

        angle %= 360;
        if (Math.abs(angle) > 180) angle -= Math.signum(angle) * 360;
        return angle;
    }

    protected static double getHeading() {

        return formatAngle((-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle % 360) - offset);
    }

    protected static void writeTelemetry(Object status) {

        staticTelemetry.addData(status.toString(), "");
        staticTelemetry.update();
    }

    double calculateInches() {

        if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
        else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
        else return CRYPTOBOX_CENTER_DISTANCE;
    }

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "init");
        telemetry.update();
        //these names are set in the configuration on the Robot Controller phone
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        relicExtension = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");
        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");


        //one set of motors has to be reversed because they are facing a different way
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        collectionMotor.setDirection(DcMotor.Direction.FORWARD);
        relicExtension.setDirection(DcMotor.Direction.FORWARD);
        deliveryMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //on the red board, the robot will be facing backwards, so we want to reverse the motors
        if (startingPosition().isRed()) {

            reverseDriveBase();
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

        ALL_MOTORS[0] = frontLeftMotor;
        ALL_MOTORS[1] = frontRightMotor;
        ALL_MOTORS[2] = backLeftMotor;
        ALL_MOTORS[3] = backRightMotor;

        IMU_Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU_Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(IMU_Parameters);

        {

            staticTelemetry = telemetry;
        }

        if (isAutonomous()) { //below is Autonomous-only code, this removes unnecessary loading

            vuforiaParameters.vuforiaLicenseKey = "AdDqKyD/////AAAAGQ/rpKTVlUiMmdwxDFRT5LiD8kI3QucN9xL8BbQRw/rYsleEjKBm/GOQB4GnSmvyzTFNFOBfZQ9o06uen5gYZvJthDx8OSVm78QegaFqHEGPjDRtqIAuLxz+12HVQXbIutqXfR595SNIl0yKUbbXFTq21ElXEDDNwO0Lv8ptnJPLib85+omkc5c8xfG6oNIhFg+sPIfCrpFACHdrr23MpY8AzLHiYleHnhpyY/y/IqsXw7CYPV2kKY70GEwH8I0MGxBw8tw8EoYpXk4vxUzHAPfgvBDztFz3x9fpcxoeqb0jl2L7GB7Aq7u+Sea+g4FoTG/9FD4rEy4I/Lz+OjdbE2eEUCGnyy10Q5o3AGG5R3cW";
            //license key set, necessary for vuforia code to be used
            vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //uses front camera of Robot Controller for detection
            //if above code is changed to ...CameraDirection.BACK;, the back-facing camera will be used instead

            //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            //if above code is enabled, the Driver Station will display the camera reading from the Robot Controller on its screen

            vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters); //vuforia object initialized based on set vuforiaParameters
            relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTrackables.activate(); //start listening for camera's data
        }
        waitForStart();
        elapsedTime.reset();
        boolean isFinished = false;
        while (opModeIsActive()) {
            if (!isFinished) {
                driveWithEncoders(DRIVE_POWER, 20.0);
                elapsedTime.reset();
                while (elapsedTime.time() <= 3) {
                    collectionMotor.setPower(0.7);
                }
                collectionMotor.setPower(0);
                elapsedTime.reset();
                while (elapsedTime.time() <= 3) {
                    setMotorsPowers(0.4, DRIVE_BASE_MOTORS);
                }
                driveWithEncoders(DRIVE_POWER, -5);
                isFinished = true;
            }
        }
    }
}
