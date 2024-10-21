package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

public class DriveTrain2 extends SubsystemBase {

    @Override
    public void periodic(){
        readSensors();
    }

    public DcMotorEx frontLeft;
    public DcMotor frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotor yEncoder;
    public DcMotor xEncoder;
    public DistanceSensor distanceSensor;
    public BHI260IMU gyro;
    private LinearOpMode currentOpMode;
    public double powerMod = 0.9;

    public double TICKS_TO_INCHES;

    public double odometryOffsetX = 0;
    public double odometryOffsetY = 0;
    public double angleOffset = 0;

    public double rawAngle;
    public double rawPosX;
    public double rawPosY;

    public double rollingPosX;
    public double rollingPosY;
    public double rollingAngle;

    public DriveTrain2(HardwareMap hardwareMap, OpMode opMode){
        this.currentOpMode = currentOpMode;

        frontLeft = hardwareMap.get(DcMotorEx .class, "frontleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backRight= hardwareMap.get(DcMotorEx.class, "backright");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");

        yEncoder = hardwareMap.get(DcMotor.class, "YENCODER");
        xEncoder = hardwareMap.get(DcMotor.class, "XENCODER");

        gyro = hardwareMap.get(BHI260IMU.class, "imu");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initGyro();
        initMotorEncoders();
    }

    public void readSensors(){
        rawPosX = xEncoder.getCurrentPosition();
        rawPosY = yEncoder.getCurrentPosition();
        rawAngle = getAngle();

        rollingAngle = rawAngle - angleOffset;
        rollingPosX = rawPosX - odometryOffsetX;
        rollingPosY = rawPosY - odometryOffsetY;
    }

    public void resetOdometry(){
        odometryOffsetX = rawPosX;
        odometryOffsetY = rawPosY;
    }

    public void resetGyro(){
        angleOffset = getAngle();
    }

    public int[] getPosTicks(){
        int xPos = xEncoder.getCurrentPosition();
        int yPos = yEncoder.getCurrentPosition();

        return new int[]{xPos, yPos};
    }

    public double[] getPosInches(){
        double xPosInches = getPosTicks()[0] * TICKS_TO_INCHES;
        double yPosInches = getPosTicks()[1] * TICKS_TO_INCHES;
        return new double[]{xPosInches, yPosInches};
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

    public void drive(double x, double y, double r, double powerMod) {
        frontLeft.setPower(powerMod * (y + x + r));
        frontRight.setPower(powerMod * (-y + x +r));
        backRight.setPower(powerMod * (-y - x + r));
        backLeft.setPower(powerMod * (y - x + r));

    }

    public void driveFieldCentric(double x, double y, double r){
        double angle = Math.toRadians(getAngle());

        //x' = xcos(theta) - ysin(theta)
        //y' = xsin(theta) + ycos(theta)

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

    public double ticksToinches(double ticks){
        return (double) -0.00000482954539 * (ticks * ticks) + 0.0503731 * ticks + 3.15417;
    }

    public double getAngle() {
        Orientation orients = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orients.firstAngle;
    }

}
