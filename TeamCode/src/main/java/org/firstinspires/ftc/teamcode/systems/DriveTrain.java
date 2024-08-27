package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain extends SubsystemBase {

    private MotorEx frontLeft;
    private MotorEx frontRight;
    private MotorEx backLeft;
    private MotorEx backRight;
    private DistanceSensor distanceSensor;
    private BNO055IMU gyro;

    Gamepad ethan = new Gamepad();
    Gamepad mica = new Gamepad();


    private MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);


    public DriveTrain(HardwareMap hardwareMap, LinearOpMode opMode) {

        frontLeft = hardwareMap.get(MotorEx.class, "frontleft");
        frontRight = hardwareMap.get(MotorEx.class, "frontRight");
        backLeft = hardwareMap.get(MotorEx.class, "backRight");
        backRight = hardwareMap.get(MotorEx.class, "BackLeft");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        frontLeft.setInverted(false);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        backRight.setInverted(true);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }



    private double detectedDistance = distanceSensor.getDistance(DistanceUnit.CM);

    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro.initialize(parameters);
    }

    private double angAdjust = 0;
    public double getAngle() {
        Orientation orients = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orients.firstAngle - angAdjust;
    }

    public void drive(double x, double y, double r) {
         frontLeft.set((y+x+r));
         frontRight.set((y-x-r));
         backRight.set((y-x+r));
         backLeft.set((y+x-r));
    }




    @Override
    public void periodic() {
        drive.driveFieldCentric(mica.left_stick_x, mica.left_stick_y, mica.right_stick_x, 0);
    }
}
