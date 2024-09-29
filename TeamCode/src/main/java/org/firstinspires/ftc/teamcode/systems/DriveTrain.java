package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain extends SubsystemBase {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DistanceSensor distanceSensor;
    public BHI260IMU gyro;
    private LinearOpMode currentOpMode;
    public double powerMod = 0.9;

    //    private double detectedDistance = distanceSensor.getDistance(DistanceUnit.CM);

//    private MecanumDrive drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
//@Override
//public void periodic(){
//
//}

    public DriveTrain(HardwareMap hardwareMap, LinearOpMode currentOpMode) {
        this.currentOpMode = currentOpMode;

        frontLeft = hardwareMap.get(DcMotorEx .class, "frontleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backRight= hardwareMap.get(DcMotorEx.class, "backright");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        gyro = hardwareMap.get(BHI260IMU.class, "imu");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initGyro();
        initMotorEncoders();
    }

    public void initMotorEncoders(){
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initGyro(){
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbFacingDirection);

        gyro.initialize(new IMU.Parameters(orientationOnRobot));
    }

//    public double referenceAngle = getAngle() - getAngle(); THIS IS THE FREEEKING PROBLEM

    public void drive(double x, double y, double r, double powerMod) {
         frontLeft.setPower(powerMod * (y+x+r));
         frontRight.setPower(powerMod * (y-x-r));
         backRight.setPower(powerMod * (y-x+r));
         backLeft.setPower(powerMod * (y+x-r));
    }

    public void driveFieldCentric(double x, double y, double r){
        double angle = Math.toRadians(getAngle()) - Math.PI;
        double adjustedX = x * Math.cos(angle) - y * Math.sin(angle);
        double adjustedY = x * Math.sin(angle) + y * Math.cos(angle);
        double[] speeds = {
                (adjustedX + adjustedY + r),
                (adjustedX - adjustedY - r),
                (adjustedX - adjustedY + r),
                (adjustedX + adjustedY - r)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) )
            {
                max = Math.abs(speeds[i]);
            }
        }

        // If the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1)
        {
            for (int i = 0; i < speeds.length; i++)
            {
                speeds[i] /= max;
            }
        }


        frontLeft.setPower(powerMod * (speeds[0]));
        frontRight.setPower(powerMod * (speeds[1]));
        backRight.setPower(powerMod * (speeds[2]));
        backLeft.setPower(powerMod * (speeds[3]));
    }

    public void driveForDistanceX(double speed, double distance){
        double targetAngle = getAngle();

        if(distance > frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            double angle = getAngle();
            double driftPval = 0.001 * (targetAngle - angle);
            drive(speed, driftPval, 0,1);
        } else if(distance < frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            drive(0,0,0,1);
        }
    }

    public void driveForDistanceY(double speed, double distance){
        double targetAngle = getAngle();
        if(distance > frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            double angle = getAngle();
            double driftPval = 0.001 * (targetAngle - angle);
            drive(0, speed, 0,1);
        } else if(distance < frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            drive(0,0,0,1);
        }
    }

    public double ticksToInches(double ticks){
        return 0.0706667-(0.0000264848 * ticks);
    }

    public double inchesToTicks(double inches){
        return (-37757.50619223 * inches) + 2668.19836283;
    }

    public double getAngle() {
        Orientation orients = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orients.firstAngle;
    }

}
