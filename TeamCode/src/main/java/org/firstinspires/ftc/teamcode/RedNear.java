package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 11/4/2017.
 */

@Autonomous(name = "RedNear")
public class RedNear extends BlueNear {

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }
}
