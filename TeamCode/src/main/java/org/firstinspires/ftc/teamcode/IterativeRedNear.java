package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by admin on 11/11/2017.
 */

@Autonomous(name = "RedNear", group = "Iterative")
public class IterativeRedNear extends IterativeBlueNear {

    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }
}
