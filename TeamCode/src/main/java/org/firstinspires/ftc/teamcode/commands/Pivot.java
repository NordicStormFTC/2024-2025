package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class Pivot extends CommandBase {

    double p = 0;
    double d = 0;
    public double target;
    double powerMod = 0;
    // this should really be angular acceleration not velocity buuuuuuut...
    public double velocity;

    // sets the error tolerance
    private final double tolerance = 1;

    public DriveTrain driveTrain;
    public double error;
    private ElapsedTime timer = new ElapsedTime();

    public Pivot(double target, DriveTrain driveTrain){
        this.target = target;
        this.driveTrain = driveTrain;
        driveTrain.resetAngle();
        timer.reset();

    }

    @Override
    public void initialize() {
        // im not sure why we cant get the initial angle here but we cant. line 37 in the constructor does it
    }

    @Override
    public void execute() {
        // error is calculated but negative because the gyro is inverted
        error = -(target - driveTrain.getAngle());

        if(Math.abs(error) < 10 && Math.abs(error) > 3){
            p = error * 0.057;
            powerMod = 0.9;
        } else if(Math.abs(error) > 10){
            p = error * 0.015;
            powerMod = 0.65;
        } else  {
            p = error * 0.186;
            powerMod = 1;
        }
        double power = p;

        driveTrain.frontLeft.setPower(power * powerMod);
        driveTrain.frontRight.setPower(power * powerMod);
        driveTrain.backLeft.setPower(power * powerMod);
        driveTrain.backRight.setPower(power * powerMod);



       // driveTrain.drive(0,0, power,powerMod);
    }

    @Override
    public boolean isFinished() {
        // this starts the timer when the robot enters the error tolerance
        // the robot must then stay within this tolerance for a certain amount of time
        // if the robot exits the error tolerance, the timer restarts
            if(Math.abs(error) > tolerance){
                timer.reset();
            }

         //return Math.abs(error) < tolerance && timer.milliseconds() > 1500;
        return timer.milliseconds() > 600;
    }

    @Override
    public void end(boolean interupted) {
       driveTrain.drive(0,0,0,0);
    }
}
