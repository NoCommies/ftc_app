package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/7/2017.
 */

public enum RobotPosition {

    RED_NEAR(Color.RED, RelativePosition.NEAR),
    RED_FAR(Color.RED, RelativePosition.FAR),
    BLUE_NEAR(Color.BLUE, RelativePosition.NEAR),
    BLUE_FAR(Color.BLUE, RelativePosition.FAR);

    private final Color color;
    private final RelativePosition position;

    RobotPosition(Color color, RelativePosition position) {

        this.color = color;
        this.position = position;
    }

    public boolean isRed() {

        return this.color == Color.RED;
    }

    public boolean isBlue() {

        return this.color == Color.BLUE;
    }

    public boolean isNear() {

        return this.position == RelativePosition.NEAR;
    }

    public boolean isFar() {

        return this.position == RelativePosition.FAR;
    }
}

enum RelativePosition {//starting position of the robot -- whether it is close or far from
    // relic scoring zone
    NEAR,
    FAR,
}

enum Color { //starting position of the robot -- its alliance color
    RED,
    BLUE;

    Color invert() {

        return this == RED ? BLUE : RED;
    }
}


