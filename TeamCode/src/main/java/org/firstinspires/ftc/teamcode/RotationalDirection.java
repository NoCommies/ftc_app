package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/14/2017.
 */

public enum RotationalDirection {

    CLOCKWISE,
    COUNTER_CLOCKWISE;

    public RotationalDirection invert() {

        return this == CLOCKWISE ? COUNTER_CLOCKWISE : CLOCKWISE;
    }
}

