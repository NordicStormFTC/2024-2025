package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class DriveForInchesY extends CommandBase {

    DriveTrain driveTrain;
    double yTarget;
    double ypower;
    double yError;
    double xTarget = 0;
    double xPower;
    double xError;
    double angularError;
    double xOdoOffset = 0;
    double yOdoOffset = 0;
    ElapsedTime timer = new ElapsedTime();
    double tolerance;
    double holdTimeMillis;
    double yP = 0;
    double yD = 0;

    public DriveForInchesY(double inches, DriveTrain driveTrain, double holdTimeMillis){
        this.driveTrain = driveTrain;
        this.yTarget = inches;
        yTarget *= 505.4334;
        this.holdTimeMillis = holdTimeMillis;

    }

    @Override
    public void initialize() {
        tolerance = 0.12 * driveTrain.TICKS_PER_INCH; // quarter inch tolerance
        yOdoOffset = driveTrain.backDriveEncoder.getCurrentPosition();
        xOdoOffset = driveTrain.strafeEncoder.getCurrentPosition();
        driveTrain.resetAngle();
        timer.reset();
    }

    @Override
    public void execute() {
        double yRollingPos = (driveTrain.backDriveEncoder.getCurrentPosition()) - yOdoOffset;
        yError = yTarget - yRollingPos;
        double acceleration = driveTrain.imu.getAcceleration().xAccel;

        if(yError > 25 * driveTrain.TICKS_PER_INCH){
            yP = 0.00004 * yError;
            yD = 0.4 * acceleration;
        }
        else if(yError > 3 * driveTrain.TICKS_PER_INCH && yError < 25 * driveTrain.TICKS_PER_INCH) {
            yP = 0.00006 * yError;
            yD = 0.12 * acceleration;
        } else if(yError < 3 * driveTrain.TICKS_PER_INCH) {
            yP = 0.0001 * yError;
            yD = (0.3 * acceleration);
        }

        if(yP > 1){
            yP /= yP;
        }
        ypower = yP - yD; // uses a d term for y axis movement


        angularError = driveTrain.getAngle();
        double rPower = 0.095 * angularError; // only uses p term


        double xRollingPos = driveTrain.strafeEncoder.getCurrentPosition() - xOdoOffset;
        xError = xTarget - xRollingPos;
        xPower = 0.00001 * xError; // only uses p term for now

        driveTrain.drive(ypower,xPower,rPower,1);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(yError) < tolerance){
            timer.startTime();
            if(Math.abs(yError) > tolerance){
                timer.reset();
            }
        }
        return Math.abs(yError) < tolerance && timer.milliseconds() > holdTimeMillis && angularError < 1;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0,0,0,0);
    }
}
