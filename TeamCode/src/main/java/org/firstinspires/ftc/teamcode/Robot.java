package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by 4924_Users on 10/7/2017.
 */

public abstract class Robot extends OpMode {

    /* FIELD PARAMETERS */

    static final double CRYPTOBOX_CENTER_DISTANCE = 35.5; //distance from center of relic-side
    static final double DRIVE_POWER = 0.4;
    static final double TURN_POWER = 0.4;
    //to center of cryptobox
    private static final double CRYPTOBOX_OFFSET = 6.5; //offset of left/right areas of cryptobox
    /* ROBOT CONSTANTS*/
    private static final int ENCODER_TICKS_PER_ROTATION = 1120; //encoder counts per shaft turn
    private static final double GEAR_RATIO = 32 / 48D; //48 teeth on motor gear, 32 teeth on wheel gear
    private static final double WHEEL_DIAMETER = 4;

    /* HARDWARE */
    //declares our hardware, initialized later in init()
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; //wheel diameter * pi
    private static final double COUNTS_PER_INCH =
            (ENCODER_TICKS_PER_ROTATION * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    private static final double GYRO_TURN_TOLERANCE_DEGREES = 5;
    private static final int ENCODER_TOLERANCE = 10;
    static
    RobotPosition STARTING_POSITION;
    static DcMotor[] DRIVE_BASE_MOTORS = new DcMotor[4];
    static DcMotor[] ALL_MOTORS = new DcMotor[4];
    /* SENSORS */
    static BNO055IMU imu;

    /* NAVIGATION */
    static BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
    /* TIME */
    static ElapsedTime elapsedTime = new ElapsedTime();
    /* VUFORIA */
    //fields for camera recognition
    static RelicRecoveryVuMark vuMark; //enum set based on pictogram reading
    /* MOTORS */
    private static DcMotor frontLeftMotor;
    private static DcMotor frontRightMotor;
    private static DcMotor backLeftMotor;
    //encoder ticks per inch moved
    private static DcMotor backRightMotor;
    private static Acceleration acceleration;
    private static VuforiaLocalizer vuforia; //later initialized with (sic) vuforiaParameters
    private static VuforiaTrackables relicTrackables;
    private static VuforiaTrackable relicTemplate;

    /* STATES *//*

    enum STATE {

        DRIVE,
        TURN,
        STOP;

        STATE[] toArray() {


        }
    }*/
    // from cryptobox center in inches, should be 6.5, but exaggerated for testing
    final double CRYPTOBOX_LEFT_DISTANCE;
    final double CRYPTOBOX_RIGHT_DISTANCE;
    private VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();

    {

        STARTING_POSITION = startingPosition();
    }

    {
        msStuckDetectInit = 10000;
        msStuckDetectInitLoop = 10000;
        msStuckDetectStart = 10000;
        msStuckDetectLoop = 10000;
    }

    //initializing CRYPTOBOX_LEFT and RIGHT DISTANCE
    {
        if (startingPosition().isClose()) {

            if (startingPosition().isBlue()) { //cryptobox is reversed when colors change

                CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
                CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
            } else {

                CRYPTOBOX_LEFT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE + CRYPTOBOX_OFFSET;
                CRYPTOBOX_RIGHT_DISTANCE = CRYPTOBOX_CENTER_DISTANCE - CRYPTOBOX_OFFSET;
            }
        } else { //STARTING_POSITION.isFar()

            CRYPTOBOX_LEFT_DISTANCE = -1;
            CRYPTOBOX_RIGHT_DISTANCE = -1;
        }
    }

    static void reverse(DcMotor d) {

        d.setDirection(d.getDirection().inverted());
    }

    static void reverseDriveBase() {

        reverse(frontLeftMotor);
        reverse(frontRightMotor);
        reverse(backLeftMotor);
        reverse(backRightMotor);

    }

    static void driveWithEncoders(double movePower, double moveDistanceInInches) {

        setMotorsTargets(moveDistanceInInches, DRIVE_BASE_MOTORS);

        setMotorsModes(DcMotor.RunMode.RUN_TO_POSITION, DRIVE_BASE_MOTORS);

        while (Math.abs(DRIVE_BASE_MOTORS[0].getCurrentPosition() - DRIVE_BASE_MOTORS[0].getTargetPosition()) > ENCODER_TOLERANCE)

            setMotorsPowers(movePower, DRIVE_BASE_MOTORS);

        setMotorsPowers(0, DRIVE_BASE_MOTORS);
    }

    static void setMotorsTargets(double encoderTargetInInches, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setTargetPosition(
                    (int) (COUNTS_PER_INCH * encoderTargetInInches) + d.getCurrentPosition());

    }

    static void setMotorsPowers(double power, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setPower(power);
    }

    static void setMotorsModes(DcMotor.RunMode runMode, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setMode(runMode);
    }

    static void turn(double power, RotationalDirection direction) {

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

    @Deprecated
    static void turnUsingOffset(double turnPower, double turnAngleOffset, RotationalDirection direction) {
        //turnAngleOffset expressed in RotationalDirection off the heading
        if (turnAngleOffset < 0) turnUsingOffset(turnPower, -turnAngleOffset, direction.invert());
        if (turnAngleOffset > 180)
            turnUsingOffset(turnPower, 180 - Math.abs(turnAngleOffset - 180), direction.invert());

        double desiredHeading = 0;
        if (direction == RotationalDirection.CLOCKWISE)
            desiredHeading = getHeading() + turnAngleOffset;
        else desiredHeading = getHeading() - turnAngleOffset;

        if (Math.abs(desiredHeading) > 180) desiredHeading += desiredHeading > 0 ? -360 : 360;
        turnToPosition(turnPower, desiredHeading);
    }

    static void turnToPosition(double turnPower, double desiredHeading) {

        //desiredHeading is the angle that we want to move to, it should be -180<x<180
        setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DRIVE_BASE_MOTORS);
        if (Math.abs(desiredHeading) > 180) desiredHeading += desiredHeading > 0 ? -360 : 360;
        //we reduce desiredHeading just in case it is formatted incorrectly
        while (((getHeading() - desiredHeading) > GYRO_TURN_TOLERANCE_DEGREES)) {

            turn(turnPower, RotationalDirection.COUNTER_CLOCKWISE);
        }
        //if it is faster to turn CCW, then robot turns CCW

        while ((getHeading() - desiredHeading) < -GYRO_TURN_TOLERANCE_DEGREES) {

            turn(turnPower, RotationalDirection.CLOCKWISE);
        }
        setMotorsPowers(0, DRIVE_BASE_MOTORS);
        //defaults to CW turning
    }

    static double getHeading() {

        return -imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle % 360;
    }

    abstract RobotPosition startingPosition();

    //true for Autonomous, false for TeleOp
    abstract boolean isAutonomous();

    double calculateInches(RelicRecoveryVuMark vuMark) {

        telemetry.addData("Starting calculateInches...", "");
        try {

            if (vuMark == RelicRecoveryVuMark.LEFT) return CRYPTOBOX_LEFT_DISTANCE;
            else if (vuMark == RelicRecoveryVuMark.RIGHT) return CRYPTOBOX_RIGHT_DISTANCE;
            else if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) //UNKNOWN is grouped with CENTER because CENTER is easiest to place
                return CRYPTOBOX_CENTER_DISTANCE;

            else throw new IllegalArgumentException
                        ("vuMark is not set to LEFT, CENTER, RIGHT, or UNKNOWN");

        } catch (RuntimeException e) { //we catch RuntimeException because the method may also
            // throw a NullPointerException if vuMark is null

            return CRYPTOBOX_CENTER_DISTANCE;
        }
    }

    void writeTelemetry(Object status) {

        telemetry.addData("Status", status.toString());
        telemetry.update();
    }

    @Override
    public void init() {

        telemetry.addData("Status", "init");
        telemetry.update();

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

        if (isAutonomous()) { //below is Autonomous-only code, this removes unnecessary loading

            vuforiaParameters.vuforiaLicenseKey = "AdDqKyD/////AAAAGQ/rpKTVlUiMmdwxDFRT5LiD8kI3QucN9xL8BbQRw/rYsleEjKBm/GOQB4GnSmvyzTFNFOBfZQ9o06uen5gYZvJthDx8OSVm78QegaFqHEGPjDRtqIAuLxz+12HVQXbIutqXfR595SNIl0yKUbbXFTq21ElXEDDNwO0Lv8ptnJPLib85+omkc5c8xfG6oNIhFg+sPIfCrpFACHdrr23MpY8AzLHiYleHnhpyY/y/IqsXw7CYPV2kKY70GEwH8I0MGxBw8tw8EoYpXk4vxUzHAPfgvBDztFz3x9fpcxoeqb0jl2L7GB7Aq7u+Sea+g4FoTG/9FD4rEy4I/Lz+OjdbE2eEUCGnyy10Q5o3AGG5R3cW";
            //license key set, necessary for vuforia code to be used
            vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //uses front camera of Robot Controller for detection
            //if above code is changed to ...CameraDirection.BACK;, the back-facing camera will be used instead

            /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);*/
            //if above code is enabled, the Driver Station will display the camera reading from the Robot Controller on its screen

            vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParameters); //vuforia object initialized based on set vuforiaParameters
            relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTrackables.activate(); //start listening for camera's data
        }
    }

    @Override
    public void init_loop() {

        if (isAutonomous()) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate); //LEFT, CENTER, or RIGHT if read, UNKNOWN if undetermined reading
            telemetry.addData("vuMark", vuMark.toString()); //LEFT, CENTER, or RIGHT, useful for debugging
            telemetry.addData("Status", "init_loop");
            telemetry.update();
        }
    }

    @Override
    public void start() {

        elapsedTime.reset(); //since init() takes so long, elapsedTime is delayed, so we reset
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

        setMotorsModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER, ALL_MOTORS);
    }
}


