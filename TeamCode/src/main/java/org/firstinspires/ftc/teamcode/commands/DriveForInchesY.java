package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.systems.DriveTrain2;

public class DriveForInchesY extends CommandBase {

    DriveTrain driveTrain;
    double target;
    double power;
    double error;
    double angularError;
    double angularOffset = 0;
    double odoOffset = 0;
    ElapsedTime timer = new ElapsedTime();
    double tolerance;

    public DriveForInchesY(double target, DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        this.target = target;

    }

    @Override
    public void initialize() {
        tolerance = 4;
        odoOffset = driveTrain.frontLeft.getCurrentPosition();
        driveTrain.resetAngle();
        timer.reset();
    }

    @Override
    public void execute() {
        double rollingPos = driveTrain.frontLeft.getCurrentPosition() - odoOffset;
        error = target - rollingPos;
        double acceleration = driveTrain.imu.getAcceleration().xAccel;

        double yP = 0.04 * error;
        // if you want to drive a long distance, the p term will be very very high,
        // and subtracting the d term will do nothing to scale the power. If the p term is higher than 1,
        // we divide it by itself so that the p term can only be a maximum of 1, and the d term will
        // have a chance to affect the power output
        if(yP > 1){
            yP /= yP;
        }
        double yD = 0.01 * acceleration;
        // this is power for the y direction PID
        power = yP - yD;

        // this is the rotation PID
        angularError = driveTrain.getAngle();
        double rP = 0.08 * angularError;
        double rPower = rP;
        driveTrain.drive(0,power,rPower,1);
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(error) < tolerance){
            timer.startTime();
            if(Math.abs(error) > tolerance){
                timer.reset();
            }
        }


        return Math.abs(error) < 3 && timer.milliseconds() > 500;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.drive(0,0,0,0);
    }
}
