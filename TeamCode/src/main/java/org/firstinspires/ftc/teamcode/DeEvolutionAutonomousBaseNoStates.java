package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by 4924_Users on 9/23/2017.
 */


@Autonomous(name = "NOGYRO", group = "Sensor")
public class DeEvolutionAutonomousBaseNoStates extends RevolutionVelocityBase {

    static final float TURNING_ANGLE_MARGIN = 7.0f;
    static final int ENCODER_TARGET_MARGIN = 15;
    public static int angleOffset = 0;
    final float THROWING_TIME = 0.5f;
    final int COUNTS_PER_REVOLUTION = 1120;
    final double WHEEL_DIAMETER = 4.0f;
    final double GEAR_RATIO = 1.0f;
    final double CALIBRATION_FACTOR = 2.24f; //1.93f;
    public int stateIndex = 0;
    public int currentPathSegmentIndex = 0;
    public int lastHeadingDifference = 0;
    public boolean stateStarted = false;
    public EncoderTargets zeroEncoderTargets = new EncoderTargets(0, 0);
    public ElapsedTime elapsedTimeForCurrentSegment = new ElapsedTime();
    public ElapsedTime elapsedTimeForCurrentState = new ElapsedTime();
    public State currentState;
    public boolean isSecondBeacon = false;
    public DrivePathSegment[] currentPath = new DrivePathSegment[]{

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] cryptoBoxCenter = new DrivePathSegment[]{

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    ElapsedTime elapsedTime;
    DrivePathSegment segment = new DrivePathSegment();
    EncoderTargets currentEncoderTargets = zeroEncoderTargets;
    double countsPerInch = 0.0;

    @Override
    public void init() {

        super.init();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        countsPerInch = (COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER)) * GEAR_RATIO * CALIBRATION_FACTOR;

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        elapsedTime = new ElapsedTime();
        useRunUsingEncoders();
        new DrivePathSegment(10.0f, 0.25f, DrivePathSegment.LINEAR);

//        while (elapsedTime.time() < 5f) {
//            frontLeftMotor.setPower(0.25);
//            frontRightMotor.setPower(0.25);
//            backLeftMotor.setPower(0.25);
//            backRightMotor.setPower(0.25);
    }

    public void useRunUsingEncoders() {

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void addEncoderTarget(int leftEncoderAdder, int rightEncoderAdder) {

        currentEncoderTargets.frontLeftTarget += leftEncoderAdder;
        currentEncoderTargets.frontRightTarget += rightEncoderAdder;
        currentEncoderTargets.backLeftTarget += leftEncoderAdder;
        currentEncoderTargets.backRightTarget += rightEncoderAdder;
    }

    public void startSeg() {

        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

        useRunUsingEncoders();
        addEncoderTarget(moveCounts, moveCounts);

        if (moveCounts < 0) {

            segment.leftPower *= -1;
            segment.rightPower *= -1;
        }
    }

    public enum State {

        STATE_INITIAL,
        STATE_DRIVE,
        STATE_STOP,
    }
}

