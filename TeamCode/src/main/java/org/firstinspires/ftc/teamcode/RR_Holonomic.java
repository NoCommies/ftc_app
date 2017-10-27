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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "RR_Holonomic", group = "Iterative Opmode")

public class RR_Holonomic extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor collectionMotor = null;
    private DcMotor relicExtension = null;
    private DcMotor deliveryMotor = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        relicExtension = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        collectionMotor.setDirection(DcMotor.Direction.FORWARD);
        relicExtension.setDirection(DcMotor.Direction.FORWARD);
        deliveryMotor.setDirection(DcMotor.Direction.FORWARD);
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double collectionPower = 0.0;
        double deliveryPower = 0.0;

        //we set what to do when the motor is not given power, which is to brake completely, instead of coasting
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double drive = -gamepad1.left_stick_y;
        //drive is what direction we want to move, either forwards, backwards, or neither

        double holonomic = -gamepad1.left_stick_x;
        //holonomic is what direction we want to move sideways

        double turnRight = gamepad1.right_trigger;
        //turnRight is how much we want to turn right

        double turnLeft = gamepad1.left_trigger;
        //turnLeft is how much we want to turn left

        boolean collectionPowerUp = gamepad2.b;
        //collectionPowerUp is dependent on whether or not we want the collection to collect

        boolean collectionPowerDown = gamepad2.a;
        //collectionPowerDown is dependent on whether or not we want the collection deliver (Push downwards)

        boolean deliveryUp = gamepad1.b;
        //deliveryUp is dependent on whether or not we want the delivery to deliver

        boolean deliveryDown = gamepad1.a;
        //deliveryDown is dependent on whether or not we want the delivery to go downwards

        double linearSlide =  (gamepad2.left_stick_y);
         //this extends the linearSlide using the measuring tape to extend it

        if (collectionPowerUp) {

            //if we want it to collect, we set collectionPower to 1
            collectionPower = 1;
        } else if (collectionPowerDown) {

            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            collectionPower = -1;
        }

        if (deliveryUp) {

            //if we want it to collect, we set collectionPower to 1
            deliveryPower = -1;
        } else if (deliveryDown) {

            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            deliveryPower = 1;
        }

        //we are calculating the power to send to each different wheel, which each need their own power since it is calculated in different ways
        double frontLeftPower =drive - holonomic + turnRight - turnLeft;
        double frontRightPower =drive + holonomic - turnRight + turnLeft;
        double backRightPower =drive - holonomic - turnRight + turnLeft;
        double backLeftPower =drive + holonomic + turnRight - turnLeft;

        // Send calculated power to wheels and motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        collectionMotor.setPower(collectionPower);
        relicExtension.setPower(linearSlide);
        deliveryMotor.setPower(deliveryPower);
        // Show the elapsed game time
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Linear Slide", "Power" + linearSlide);

        turnLeft = 0;
        turnRight = 0;



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}