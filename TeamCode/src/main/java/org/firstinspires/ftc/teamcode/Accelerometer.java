package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.*;

/**
 * Created by user6 on 12/9/2017.
 */

public class Accelerometer extends AccelerometerRobot {


    public static Location startingPosition;

    public static void driveToPoint(Location targetPosition) {

        startingPosition = currentPosition();
        double targetAngle = Location.getTargetAngle(currentPosition(), targetPosition);
        turnToPosition(TURN_POWER, targetAngle);
//        driveWithEncoders(DRIVE_POWER, Location.distance(currentPosition(), targetPosition));
        setMotorsPowers(DRIVE_POWER, DRIVE_BASE_MOTORS);
        double lastLeftPower = 0;
        double lastRightPower = 0;
        while(Location.distance(currentPosition(), targetPosition) > 0.5) {

            setMotorsPowers(lastLeftPower + Location.error(startingPosition, targetAngle), LEFT_MOTORS);
            setMotorsPowers(lastRightPower - Location.error(startingPosition, targetAngle), RIGHT_MOTORS);
        }
    }

    public static final DcMotor[] LEFT_MOTORS = {frontLeftMotor, backLeftMotor};
    public static final DcMotor[] RIGHT_MOTORS = {frontRightMotor, backRightMotor};

    public final double PROPORTIONAL_COEFFICIENT = 1000;

    public static Location currentPosition() {

        return new Location(imu.getPosition().x, imu.getPosition().y);
    }

    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_FAR;
    }

    public boolean isAutonomous() {

        return true;
    }

    private static class Location {

        double x;
        double y;

        Location(double x, double y) {

            this.x = x;
            this.y = y;
        }

        static double deltaX(Location l1, Location l2) {

            return l2.x - l1.x;
        }

        static double deltaY(Location l1, Location l2) {

            return l2.x - l1.x;
        }

        static double distance(Location l1, Location l2) {

            return sqrt(pow(deltaX(l1, l2), 2) + pow(deltaY(l1, l2), 2));
        }

        static double getTargetAngle(Location l1, Location l2) {

            double hypotenuse = distance(l1, l2);
            return formatAngle(asin(hypotenuse/deltaY(l1, l2)) + getHeading());
        }

        static double error(Location startingPosition, double targetAngle) {

            return distance(startingPosition, currentPosition())*sin(targetAngle);
        }
    }

    public void runOpMode() {


    }
}
