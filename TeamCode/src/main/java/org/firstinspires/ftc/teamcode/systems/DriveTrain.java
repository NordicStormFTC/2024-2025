package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public BNO055IMU gyro;
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
        frontLeft = hardwareMap.get(DcMotorEx .class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight= hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        gyro = hardwareMap.get(BNO055IMU .class, "gyro");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initGyro();
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
         frontLeft.setPower((y+x+r));
         frontRight.setPower((y-x-r));
         backRight.setPower((y-x+r));
         backLeft.setPower((y+x-r));
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
            drive(speed, driftPval, 0);
        } else if(distance < frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            drive(0,0,0);
        }
    }

    public void driveForDistanceY(double speed, double distance){
        double targetAngle = getAngle();
        if(distance > frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            double angle = getAngle();
            double driftPval = 0.001 * (targetAngle - angle);
            drive(driftPval, speed, 0);
        } else if(distance < frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
            drive(0,0,0);
        }
    }

    public double ticksToInches(double ticks){
        return 0.0706667-(0.0000264848 * ticks);
    }

    public double inchesToTicks(double inches){
        return (-37757.50619223 * inches) + 2668.19836283;
    }
}
