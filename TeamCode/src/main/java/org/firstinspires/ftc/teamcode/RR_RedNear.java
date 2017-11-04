package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by 4924_Users on 10/13/2017.
 */

@Autonomous(name = "VumarkRedNear")
public class RR_RedNear extends RR_BlueNear {

    boolean notFinished = true;

    @Override
    public void loop() {

        super.loop();
    }

    @Override
    public RobotPosition startingPosition() {

        return RobotPosition.RED_NEAR;
    }

    public boolean isAutonomous() {

        return true;
    }
}
