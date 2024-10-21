package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrain extends SubsystemBase {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DistanceSensor distanceSensor;
    public BNO055IMU imu;
    private LinearOpMode currentOpMode;

    public double powerMod = 1;

    public double angleOffset = 0;
    public double rawAngle =0;

    // this is a holder angle for the get angle
    // function and is never returned by a function
    private Orientation lastAngle = new Orientation();

    // this is the actual rolling acumulative angle
    double globalAngle = 0;


    // the constructor initializes all hardware and the gyroscope
    // as well as sets the run mode for the motors
    public DriveTrain(HardwareMap hardwareMap, LinearOpMode currentOpMode) {
        this.currentOpMode = currentOpMode;

        frontLeft = hardwareMap.get(DcMotorEx .class, "frontleft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontright");
        backRight= hardwareMap.get(DcMotorEx.class, "backright");
        backLeft = hardwareMap.get(DcMotorEx.class, "backleft");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        // so the thing is that, one night, mecanum drive just
        // stoped working, like literaly nothing in the code changed
        // but the drive config was f***** soo....
        // yeh it was like 2 weeks of trouble shooting until I set all
        // all motors to forwards and inverted the power distribution equation
        // for the motors that should have been set to inverted. see "drive" function.
        // as evidence this is what the motor config should look like, but like no
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        // looooooove programming

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // these functions seem self explanatory but DM me ig
        initGyro();
        initMotorEncoders();
    }

    public void readSensors(){
        rawAngle = getAngle();
//        rollingAngle = rawAngle - angleOffset;
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

    }

    // so this is where the power distribution equation is inverted to make mecanum drive work
    public void drive(double x, double y, double r, double powerMod) {
         frontLeft.setPower(powerMod * (y + x + r));
         frontRight.setPower(powerMod * (-y + x +r));
         backRight.setPower(powerMod * (-y - x + r));
         backLeft.setPower(powerMod * (y - x + r));

    }

    public void driveFieldCentric(double x, double y, double r, double powerMod){
       // by subtracting PI we just set forwards to the front of the robot
        double angle = Math.toRadians(getAngle()) - Math.PI;

        // this is where we transform the angles
        //x' = xcos(theta) - ysin(theta)
        //y' = xsin(theta) + ycos(theta)
        double adjustedX = x * Math.cos(angle) - y * Math.sin(angle);
        double adjustedY = x * Math.sin(angle) + y * Math.cos(angle);

        double[] speeds = {
                (adjustedY + adjustedX + r),
                (-adjustedY + adjustedX + r),
                (-adjustedY - adjustedX + r),
                (adjustedY - adjustedX + r)
        };

        //so like if the power requested is more than 1, we devide the number by
        // itself to normalize the power request
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

    // we do this in a command now
//    public void driveForDistanceX(double speed, double distance){
//        double targetAngle = getAngle();
//
//        if(distance > frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
//            double angle = getAngle();
//            double driftPval = 0.001 * (targetAngle - angle);
//            drive(speed, driftPval, 0,1);
//        } else if(distance < frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
//            drive(0,0,0,1);
//        }
//    }

    // we do this in a command now
//    public void driveForDistanceY(double speed, double distance){
//        double targetAngle = getAngle();
//        if(distance > frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
//            double angle = getAngle();
//            double driftPval = 0.001 * (targetAngle - angle);
//            drive(0, speed, 0,1);
//        } else if(distance < frontLeft.getCurrentPosition() && currentOpMode.opModeIsActive()){
//            drive(0,0,0,1);
//        }
//    }

    // Bogus
    public double ticksToInches(double ticks){
        return 0.0706667-(0.0000264848 * ticks);
    }

    // Nope
    public double inchesToTicks(double inches){
        return (-37757.50619223 * inches) + 2668.19836283;
    }

    // this resets the rolling accumulative angle of imu
    public void resetAngle(){
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {

        //ok so this was like the old way of doing it,
        // but it only returned a value between 180 and -180 and this was a problem
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        Orientation orients = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return orientation.getYaw(AngleUnit.DEGREES);


        //so this way returns the actual angle
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

}
