
package NoCommies;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NewNaiveAccelerationIntegrator;


/**
 * Created by user6 on 12/9/2017.
 */


public abstract class NewRobot extends LinearOpMode {

    static DcMotor frontLeftMotor;
    static DcMotor frontRightMotor;
    static DcMotor backLeftMotor;
    static DcMotor backRightMotor;
    static DcMotor collectionMotor;
    static DcMotor relicExtension;
    static DcMotor deliveryMotor;

    static Servo barServo;
    static CRServo elbowServo;
    static CRServo clawServo;
    static Servo jewelArmXAxis;
    static Servo jewelArmYAxis;
    static Servo alignmentDevice;

    static BNO055IMU imu;
    static BNO055IMU.Parameters imuParameters;

    /*public <T> void setMotorsAttributes(Consumer<T> applicator, T attribute, DcMotor[] manipulants) {

        for(DcMotor manipulant : manipulants) {

            manipulant.applicator.accept(attribute); //TODO: confirm that the code works once language level updated
        }
    } //TODO: update language level to java 1.8
*/


    protected void Init() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        collectionMotor = hardwareMap.get(DcMotor.class, "collectionMotor");
        relicExtension = hardwareMap.get(DcMotor.class, "relicExtension");
        deliveryMotor = hardwareMap.get(DcMotor.class, "deliveryMotor");

        elbowServo = hardwareMap.get(CRServo.class, "elbowServo");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParameters.accelerationIntegrationAlgorithm = new NewNaiveAccelerationIntegrator();

    }

    protected void Init_Loop() {}
    protected void Start() {}
    protected void Loop() {}
    protected void Stop() {}
}
