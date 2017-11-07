package ExperimentalClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotPosition;

/**
 * Created by 4924_Users on 9/30/2017.
 */
@Disabled
@Autonomous(name = "EncoderTurningTest")
public class EncoderTestProgram extends Robot {

    boolean notFinished = true;

    @Override
    public void start() {

        super.start();
    }

    @Override
    public void loop() {

        super.loop();
        if (notFinished) {
            /*
            driveWithEncoders(DRIVE_POWER, calculateInches());
            turnToPosition(TURN_POWER, -90);
            setMotorsPowers(0, DRIVE_BASE_MOTORS);
            driveWithEncoders(DRIVE_POWER, 5);
            collectionMotor.setPower(-1);
            */
            turnWithEncoders(TURN_POWER, -17);
            turnWithEncoders(TURN_POWER, 17);
            turnWithEncoders(TURN_POWER, -18);
            turnWithEncoders(TURN_POWER, 18);
            turnWithEncoders(TURN_POWER, -19);
            turnWithEncoders(TURN_POWER, 19);
            turnWithEncoders(TURN_POWER, -20);
            turnWithEncoders(TURN_POWER, 20);

            notFinished = false;
        }
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.BLUE_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
