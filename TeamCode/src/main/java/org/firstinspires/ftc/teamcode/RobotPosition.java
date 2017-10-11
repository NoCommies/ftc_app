package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 10/7/2017.
 */

public enum RobotPosition {
    RED_CLOSE(Color.RED, Position.CLOSE),
    RED_FAR(Color.RED, Position.FAR),
    BLUE_CLOSE(Color.BLUE, Position.CLOSE),
    BLUE_FAR(Color.BLUE, Position.FAR);

    private final Color color;
    private final Position position;

    RobotPosition(Color color, Position position) {

        this.color = color;
        this.position = position;
    }

    public boolean isRed() {

        return color == Color.RED;
    }

    public boolean isBlue() {

        return color == Color.BLUE;
    }

    public boolean isClose() {

        return position == Position.CLOSE;
    }

    public boolean isFar() {

        return position == Position.FAR;
    }
}
