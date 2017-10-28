package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/27/2017.
 */

@Autonomous(name = "VumarkRedFar")
public class VumarkRedFar extends VumarkBlueFar {

    @Override
    RobotPosition startingPosition() {

        return RobotPosition.RED_FAR;
    }
}
