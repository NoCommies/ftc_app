package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/27/2017.
 */

@Autonomous(name = "RR_RedFar")
public class RR_RedFar extends RR_BlueFar {

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.RED_FAR;
    }
}
