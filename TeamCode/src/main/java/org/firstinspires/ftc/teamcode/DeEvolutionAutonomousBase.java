package org.firstinspires.ftc.teamcode;

/**
 * Created by 4924_Users on 1/3/2017.
 */
/*
public abstract class DeEvolutionAutonomousBase extends RevolutionVelocityBase {

    public enum State {

        STATE_INITIAL,
        STATE_STOP,
        STATE_READ_PICTOGRAPH,
        STATE_PLACE_GLYPH,
        STATE_DRIVE_TO_CRYPTOBOX,
    }

    final float THROWING_TIME = 0.5f;
    public int stateIndex = 0;
    public int currentPathSegmentIndex = 0;
    public int lastHeadingDifference = 0;
    public boolean stateStarted = false;
    DrivePathSegment segment = new DrivePathSegment();
    public EncoderTargets zeroEncoderTargets = new EncoderTargets(0, 0);
    EncoderTargets currentEncoderTargets = zeroEncoderTargets;

    public DrivePathSegment[] currentPath = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] stop = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    public DrivePathSegment[] cryptoBoxLeft = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] cryptoBoxRight = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };
    public DrivePathSegment[] cryptoBoxCenter = new DrivePathSegment[] {

            new DrivePathSegment(0.0f, 0.0f, DrivePathSegment.LINEAR),
    };

    double countsPerInch = 0.0;
    public ElapsedTime elapsedTimeForCurrentSegment = new ElapsedTime();
    public ElapsedTime elapsedTimeForCurrentState = new ElapsedTime();
    public State currentState;
    static final float TURNING_ANGLE_MARGIN = 7.0f;
    static final int ENCODER_TARGET_MARGIN = 15;
    public static int angleOffset = 0;
    final int COUNTS_PER_REVOLUTION = 1120;
    final double WHEEL_DIAMETER = 4.0f;
    final double GEAR_RATIO = 1.0f;
    final double CALIBRATION_FACTOR = 2.24f; //1.93f;
    public boolean isSecondBeacon = false;

//    OpenGLMatrix lastLocation = null;
//    VuforiaLocalizer vuforia;
    RelicRecoveryVuMark vuMark;
    VuforiaTrackable relicTemplate;

    @Override
    public void init() {

        super.init();
        currentState = State.STATE_INITIAL;
        countsPerInch = (COUNTS_PER_REVOLUTION / (Math.PI * WHEEL_DIAMETER)) * GEAR_RATIO * CALIBRATION_FACTOR;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdDqKyD/////AAAAGQ/rpKTVlUiMmdwxDFRT5LiD8kI3QucN9xL8BbQRw/rYsleEjKBm/GOQB4GnSmvyzTFNFOBfZQ9o06uen5gYZvJthDx8OSVm78QegaFqHEGPjDRtqIAuLxz+12HVQXbIutqXfR595SNIl0yKUbbXFTq21ElXEDDNwO0Lv8ptnJPLib85+omkc5c8xfG6oNIhFg+sPIfCrpFACHdrr23MpY8AzLHiYleHnhpyY/y/IqsXw7CYPV2kKY70GEwH8I0MGxBw8tw8EoYpXk4vxUzHAPfgvBDztFz3x9fpcxoeqb0jl2L7GB7Aq7u+Sea+g4FoTG/9FD4rEy4I/Lz+OjdbE2eEUCGnyy10Q5o3AGG5R3cW";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

        telemetry.addData("currentState: ", currentState);
        telemetry.addData("segment index", currentPathSegmentIndex);


        //telemetry.addData("Target", segment.Angle);


        switch (currentState) {

            case STATE_INITIAL:

               // if (!turningGyro.isCalibrating()) {

                    //steadyHeading = heading;
                    runWithoutEncoders();
                    switchToNextState();
                //}

                break;



           case STATE_STOP:

                startPath(stop);
                TurnOffAllDriveMotors();

                break;

            case STATE_READ_PICTOGRAPH:
                while(elapsedTimeForCurrentState.time() < 15.0f || vuMark != RelicRecoveryVuMark.UNKNOWN) {

                   vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    telemetry.update();
                }
                switchToNextState();
                break;



            case STATE_PLACE_GLYPH:
                switch(vuMark) {

                    case LEFT:

                        telemetry.update();
                        startPath(cryptoBoxLeft);
                        break;

                    case UNKNOWN:

                    case CENTER:
                        startPath(cryptoBoxCenter);
                        telemetry.update();
                        break;

                    case RIGHT:
                        startPath(cryptoBoxRight);
                        telemetry.update();
                        break;

                }
           case STATE_DRIVE_TO_CRYPTOBOX:

                        startPath(cryptoBoxCenter);
                        switchToNextState();
                        break;

        }

        setMotorPowerLevels(powerLevels);
    }

    public void strafeAgainstWall(int heading) {

        /*if (leftBumper.isPressed()) {

            if (rightBumper.isPressed()) {

                if (isRed()) {

                    if (isSecondBeacon) {

                        setPowerForMecanumStrafe(-0.35f, heading);

                    } else {

                        setPowerForMecanumStrafe(-0.2f, heading);
                    }

                } else {

                    if (isSecondBeacon) {

                        setPowerForMecanumStrafe(0.35f, heading);

                    } else {

                        setPowerForMecanumStrafe(0.2f, heading);
                    }
                }

            } else {

                powerLevels.frontRightPower = 0.2f;
                powerLevels.backRightPower = 0.2f;
                powerLevels.frontLeftPower = 0.0f;
                powerLevels.backLeftPower = 0.0f;
            }

        } else {

            if (rightBumper.isPressed()) {

                powerLevels.frontRightPower = 0.0f;
                powerLevels.backRightPower = 0.0f;
                powerLevels.frontLeftPower = 0.2f;
                powerLevels.backLeftPower = 0.2f;

            } else {

                powerLevels.frontRightPower = 0.2f;
                powerLevels.backRightPower = 0.2f;
                powerLevels.frontLeftPower = 0.2f;
                powerLevels.backLeftPower = 0.2f;
            }
        }*/

  /*      if (isRed()) {

            if (isSecondBeacon) {

                setPowerForMecanumStrafe(-0.5f, heading);

            } else {

                setPowerForMecanumStrafe(-0.2f, heading);
            }

        } else {

            if (isSecondBeacon) {

                setPowerForMecanumStrafe(0.4f, heading);

            } else {

                setPowerForMecanumStrafe(0.2f, heading);
            }
        }
    }

    public void setPowerForMecanumStrafe(float power, int heading) {

        int headingDifference = steadyHeading - heading;

        if (steadyHeading - heading >= 180) {

            headingDifference = steadyHeading - 360 - heading;
        }

        if (heading - steadyHeading >= 180) {

            headingDifference = 360 - heading + steadyHeading;
        }

        if (headingDifference < 0) {

            powerLevels.frontLeftPower = -power; //+ Math.abs(headingDifference / 15);
            powerLevels.backLeftPower = power; //+ Math.abs(headingDifference / 15);
            powerLevels.backRightPower = -power;
            powerLevels.frontRightPower = power;

        } else {

            powerLevels.frontLeftPower = -power;
            powerLevels.backLeftPower = power;
            powerLevels.backRightPower = -power; //+ Math.abs(headingDifference / 15);
            powerLevels.frontRightPower = power; //+ Math.abs(headingDifference / 15);
        }
    }

    public void switchToNextState() {

        elapsedTimeForCurrentState.reset();
        stateIndex++;
        stateStarted = false;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
    }

    public void switchToNextState(int states) {

        elapsedTimeForCurrentState.reset();
        stateIndex += states;
        stateStarted = false;

        if (stateIndex >= stateList().length) {

            stateIndex = stateList().length - 1;
        }

        if (stateIndex < 0) {

            stateIndex = 0;
        }

        currentState = stateList()[stateIndex];
    }

    public void startSeg() {

        segment = currentPath[currentPathSegmentIndex];
        elapsedTimeForCurrentSegment.reset();

        int heading = turningGyro.getHeading();
        steadyHeading = heading;

        if (currentPath != null) {

            if (segment.isTurn) {

                segment.Angle += angleOffset;

                if (segment.Angle >= 360) {

                    segment.Angle -= 360;
                }

                runWithoutEncoders();
                double currentAngle = heading;
                segment.isClockwise = !counterclockwiseTurnNeeded(currentAngle);

                if (counterclockwiseTurnNeeded(currentAngle)) {

                    segment.rightPower = -segment.rightPower;

                } else {

                    segment.leftPower = -segment.leftPower;
                }

                powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

            } else {

                if (segment.isDelay) {

                    runWithoutEncoders();
                    segment.leftPower = 0.0f;
                    segment.rightPower = 0.0f;

                    powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

                } else {

                    if (segment.isHolonomic) {

                        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                        useRunUsingEncoders();
                        addEncoderTarget(moveCounts, moveCounts);

                        if (segment.RightSideDistance < 0.0f) {

                            segment.rightPower *= -1;
                            segment.leftPower *= -1;
                        }

                        setPowerForMecanumStrafe(segment.rightPower, heading);

                    } else {

                        int moveCounts = (int) (segment.LeftSideDistance * countsPerInch);

                        useRunUsingEncoders();
                        addEncoderTarget(moveCounts, moveCounts);

                        if (moveCounts < 0) {

                            segment.leftPower *= -1;
                            segment.rightPower *= -1;
                        }

                        powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);
                    }
                }
            }

            currentPathSegmentIndex++;
        }
    }

    public void startPath(DrivePathSegment[] path) {

        currentPath = path;
        currentPathSegmentIndex = 0;
        setEncoderTargetsToCurrentPosition();
        useRunUsingEncoders();
        startSeg();
    }

    public boolean pathComplete(int heading) {
        // Wait for this Segment to end and then see what's next.
        if (segmentComplete(heading)) {
            // Start next Segment if there is one.
            if (currentPathSegmentIndex < currentPath.length) {

                TurnOffAllDriveMotors();
                startSeg();

            } else {

                currentPath = null;
                currentPathSegmentIndex = 0;
                TurnOffAllDriveMotors();
                return true;
            }

        } else {

            if (!segment.isTurn) {

                if (segment.isHolonomic) {

                    setPowerForMecanumStrafe(segment.rightPower, turningGyro.getHeading());

                } else {

                    if (segment.isDelay) {

                        TurnOffAllDriveMotors();

                    } else {

                        setPowerForLinearMove(segment.rightPower);
                    }
                }
            }
        }

        return false;
    }

    public int getFrontRightPosition() {

        return frontRightMotor.getCurrentPosition();
    }

    public int getFrontLeftPosition() {

        return frontLeftMotor.getCurrentPosition();
    }

    public int getBackRightPosition() {

        return backRightMotor.getCurrentPosition();
    }

    public int getBackLeftPosition() {

        return backLeftMotor.getCurrentPosition();
    }

    public void setEncoderTargetsToCurrentPosition() {

        currentEncoderTargets.frontLeftTarget = getFrontLeftPosition();
        currentEncoderTargets.frontRightTarget = getFrontRightPosition();
        currentEncoderTargets.backLeftTarget = getBackLeftPosition();
        currentEncoderTargets.backRightTarget = getBackRightPosition();
    }

    public boolean segmentComplete(int heading) {

        if (segment.isTurn) {

            if (turnComplete()) {

                return true;

            } else {

                int headingDifference = (int) segment.Angle - heading;

                if ((int) segment.Angle - heading >= 180) {

                    headingDifference = (int) segment.Angle - 360 - heading;
                }

                if (heading - (int) segment.Angle >= 180) {

                    headingDifference = 360 - heading + (int) segment.Angle;
                }

                if (segment.isClockwise) {

                    if (headingDifference < 0) {

                        segment.leftPower *= -1;
                        segment.rightPower *= -1;
                        segment.isClockwise = false;
                    }

                } else {

                    if (headingDifference > 0) {

                        segment.leftPower *= -1;
                        segment.rightPower *= -1;
                        segment.isClockwise = true;
                    }
                }

                powerLevels = new PowerLevels(segment.leftPower, segment.rightPower, segment.leftPower, segment.rightPower);

                telemetry.addData("HD" , headingDifference);
                telemetry.addData("LHD", lastHeadingDifference);

                return false;
            }

        } else {

            if (segment.isDelay) {

                return delayComplete();

            } else {

                return linearMoveComplete();
            }
        }
    }

    public boolean turnComplete() {

        int heading = turningGyro.getHeading();

        return Math.abs(segment.Angle) <= heading + TURNING_ANGLE_MARGIN &&
                Math.abs(segment.Angle) >= heading - TURNING_ANGLE_MARGIN;
    }

    public boolean delayComplete() {

        return elapsedTimeForCurrentSegment.time() >= segment.delayTime;
    }

    public boolean linearMoveComplete() {

        int frontLeftTarget = currentEncoderTargets.frontLeftTarget;
        int frontRightTarget = currentEncoderTargets.frontRightTarget;
        int backLeftTarget = currentEncoderTargets.backLeftTarget;
        int backRightTarget = currentEncoderTargets.backRightTarget;

        return (//isPositionClose(getFrontRightPosition(), frontRightTarget) ||
                //isPositionClose(getFrontLeftPosition(), frontLeftTarget) ||
                isPositionClose(getBackRightPosition(), backRightTarget) ||
                isPositionClose(getBackLeftPosition(), backLeftTarget)) ||
                (//isPastTarget(getFrontRightPosition(), frontRightTarget, segment.RightSideDistance) ||
                        //isPastTarget(getFrontLeftPosition(), frontLeftTarget, segment.LeftSideDistance) ||
                        isPastTarget(getBackRightPosition(), backRightTarget, segment.RightSideDistance) ||
                        isPastTarget(getBackLeftPosition(), backLeftTarget, segment.LeftSideDistance));
    }

    public boolean isPositionClose(int position, int target) {

        return Math.abs(target - position) < ENCODER_TARGET_MARGIN;
    }

    public boolean isPastTarget(int position, int target, float distanceToMove) {

        if (distanceToMove < 0) {

            return position < target;
        }

        return position > target;
    }


    public void addEncoderTarget(int leftEncoderAdder, int rightEncoderAdder) {

        currentEncoderTargets.frontLeftTarget += leftEncoderAdder;
        currentEncoderTargets.frontRightTarget += rightEncoderAdder;
        currentEncoderTargets.backLeftTarget += leftEncoderAdder;
        currentEncoderTargets.backRightTarget += rightEncoderAdder;
    }

    public boolean counterclockwiseTurnNeeded(double currentAngle) {

        telemetry.addData("Angle: ", currentAngle);

        if (currentAngle < Math.abs(segment.Angle)) {

            return (Math.abs(segment.Angle) - currentAngle) >= 180.0f;
        }

        return (currentAngle - Math.abs(segment.Angle)) <= 180.0f;
    }

    public void useRunUsingEncoders() {

        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void lowerAutoThrowingArm(ElapsedTime ElapsedThrowingTime, float throwingTime) {

        if (ElapsedThrowingTime.time() >= throwingTime * 1.5) {

            throwingArmPowerLevel = 0.0f;

        } else {

            throwingArmPowerLevel = -0.8f;
        }
    }

    //public abstract boolean isRed();

    public abstract State[] stateList();
}
*/