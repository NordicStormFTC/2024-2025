package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
    public BNO055IMU imu;
    private LinearOpMode currentOpMode;
    public double powerMod = 0.9;

    private Orientation lastAngle = new Orientation();

    // this is the actual rolling accumulative angle
    double globalAngle = 0;


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

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initGyro();
        initMotorEncoders();
    }

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

    }

    public void resetAngle(){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngle.firstAngle;

        if(deltaAngle < -180){
            deltaAngle +=360;
        } else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngle = angles;

        return globalAngle;
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

    public void drive(double x, double y, double r, double powerMod) {
        frontLeft.setPower(powerMod * (y + x + r));
        frontRight.setPower(powerMod * (-y + x +r));
        backRight.setPower(powerMod * (-y - x + r));
        backLeft.setPower(powerMod * (y - x + r));

    }
}
