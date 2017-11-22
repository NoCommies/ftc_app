package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by admin on 11/11/2017.
 */

@Autonomous(name = "RedFar", group = "Iterative")
public class IterativeRedFar extends IterativeBlueFar {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_FAR;
    }
}
