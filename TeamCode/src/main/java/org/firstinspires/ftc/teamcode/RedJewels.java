package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 4924_Users on 11/22/2017.
 */
@Autonomous(name = "Red Jewels", group = "Iterative")
public class RedJewels extends LinearOpMode {

    ColorSensor sensorColor;
    private Servo armY = null;
    private Servo armX = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "init");
        telemetry.update();
        //these names are set in the configuration on the Robot Controller phone
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        armY = hardwareMap.get(Servo.class, "armY");
        armX = hardwareMap.get(Servo.class, "armX");
        boolean jewelDone = false;

        armX.setPosition(0.5);
        armY.setPosition(1);
        // wait for the start button to be pressed.
        waitForStart();
        ElapsedTime opmodeRunTime = new ElapsedTime();
        armY.setPosition(0.55);
        while (opModeIsActive()) {
            if (opmodeRunTime.seconds() > 3 && !jewelDone) {
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.update();

                if (sensorColor.red() < sensorColor.blue()) {
                    armX.setPosition(0.625);
                    jewelDone = true;
                    telemetry.addLine("Moving X-Axis; Color Blue");
                    telemetry.update();
                } else if (sensorColor.blue() < sensorColor.red()) {
                    armX.setPosition(0.325);
                    jewelDone = true;
                    telemetry.addLine("Moving X-Axis; Color Red");
                    telemetry.update();
                } else {
                    telemetry.addLine("Too Close To Tell");
                    jewelDone = true;
                    telemetry.update();
                }
            }
            if (jewelDone){
                armX.setPosition(0.5);
                armY.setPosition(1);
            }
        }
    }
}
