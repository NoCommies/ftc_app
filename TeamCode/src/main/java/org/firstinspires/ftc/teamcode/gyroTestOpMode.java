/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "gyroTestOpMode", group = "Sensor")
public class gyroTestOpMode extends DeEvolutionAutonomousBase {

    @Override
    public State[] stateList() {

        return new State[]{

                State.STATE_INITIAL,
               // State.STATE_READ_PICTOGRAPH,
//                State.STATE_PLACE_GLYPH,
                State.STATE_DRIVE_TO_CRYPTOBOX,
                State.STATE_STOP,
        };
    }
    public gyroTestOpMode(){

        cryptoBoxCenter = new DrivePathSegment[] {

                new DrivePathSegment(2.85f, 0.25f, DrivePathSegment.LINEAR),
                new DrivePathSegment(-90.0f, 0.5f, DrivePathSegment.TURN),
                new DrivePathSegment(1.0f, 0.25f, DrivePathSegment.LINEAR),
        };
    }

    @Override
    public void loop() {

        telemetry.update();

    }

}
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

//    // The IMU sensor object
//    BNO055IMU imu;
//
//    // State used for updating telemetry
//    Orientation angles;
//    Acceleration gravity;
//
//    //----------------------------------------------------------------------------------------------
//    // Main logic
//    //----------------------------------------------------------------------------------------------
//
//    @Override
//    public void runOpMode() {
//
//        final double MOVE_POWER = 0.25;
//
//        //instantiating motors
//        DcMotor frontLeftMotor;
//        DcMotor frontRightMotor;
//        DcMotor backLeftMotor;
//        DcMotor backRightMotor;
//
//        //initializing motors
//        frontLeftMotor = hardwareMap.get(DcMotor.class, "flm");
//        frontRightMotor = hardwareMap.get(DcMotor.class, "frm");
//        backLeftMotor = hardwareMap.get(DcMotor.class, "blm");
//        backRightMotor = hardwareMap.get(DcMotor.class, "brm");
//
//        //setting motor direction
//        //2 motors are reversed because they are facing different directions
//        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        // Set up the parameters with which we will use our IMU. Note that integration
//        // algorithm here just reports accelerations to the logcat log; it doesn't actually
//        // provide positional information.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        // Set up our telemetry dashboard
//        composeTelemetry();
//
//        // Wait until we're told to go
//        waitForStart();
//
//        // Start the logging of measured acceleration
//        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
//
//        // Loop and update the dashboard
//        while (opModeIsActive()) {
//            telemetry.update();
//            while (gamepad1.y) { //y button on driver 1 is pressed
//
//                while (angles.firstAngle > 2) { //facing 2 degrees toward the right
//                    //turn left
//                    frontLeftMotor.setPower(-MOVE_POWER);
//                    backLeftMotor.setPower(-MOVE_POWER);
//                    frontRightMotor.setPower(MOVE_POWER);
//                    backRightMotor.setPower(MOVE_POWER);
//                }
//
//                while (angles.firstAngle < -2) { //facing 2 degrees toward the left
//                    //turn right
//                    frontLeftMotor.setPower(MOVE_POWER);
//                    backLeftMotor.setPower(MOVE_POWER);
//                    frontRightMotor.setPower(-MOVE_POWER);
//                    backRightMotor.setPower(-MOVE_POWER);
//                }
//
//                while (Math.abs(angles.firstAngle) <= 2) { //doesn't meet other 2 conditions
//                    //drive straight forward
//                    frontLeftMotor.setPower(MOVE_POWER);
//                    backLeftMotor.setPower(MOVE_POWER);
//                    frontRightMotor.setPower(MOVE_POWER);
//                    backRightMotor.setPower(MOVE_POWER);
//                }
//            }
//
//            if(gamepad1.x && !gamepad1.y) {
//
//                frontLeftMotor.setPower(0);
//                frontRightMotor.setPower(0);
//                backLeftMotor.setPower(0);
//                backRightMotor.setPower(0);
//            }
//        }
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // Telemetry Configuration
//    //----------------------------------------------------------------------------------------------
//
//    void composeTelemetry() {
//
//        // At the beginning of each telemetry update, grab a bunch of data
//        // from the IMU that we will then display in separate lines.
//        telemetry.addAction(new Runnable() {
//            @Override
//            public void run() {
//                // Acquiring the angles is relatively expensive; we don't want
//                // to do that in each of the three items that need that info, as that's
//                // three times the necessary expense.
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//            }
//        });
//
//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override
//                    public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel * gravity.xAccel
//                                        + gravity.yAccel * gravity.yAccel
//                                        + gravity.zAccel * gravity.zAccel));
//                    }
//                })
//                .addData("Y is pressed", gamepad1.y)
//                .addData("Updated", true);
//
//    }
//
//    //----------------------------------------------------------------------------------------------
//    // Formatting
//    //----------------------------------------------------------------------------------------------
//
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
//
//    String formatDegrees(double degrees) {
//        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
//    }

*/