package org.firstinspires.ftc.teamcode.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class Pivot extends CommandBase {


    public double target;
    // this should really be angular acceleration not velocity buuuuuuut...
    public double velocity;

    // sets the error tolerance
    private final double tolerance = 3;

    public DriveTrain driveTrain;
    public double error;
    private ElapsedTime timer = new ElapsedTime();

    public Pivot(double target, DriveTrain driveTrain){
        // target is negative because gyro is inverted
        this.target = -target;
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

        //takes the angular velocity of the imu .xRotationRate makes it a double instead of
        // an angular velocity object
        velocity = - driveTrain.imu.getAngularVelocity().xRotationRate;
        double p = error * .08;
        double d = velocity * .005;

        double power = p - d;

        driveTrain.drive(0,0, power,1);
    }

    @Override
    public boolean isFinished() {
        // this starts the timer when the robot enters the error tolerance
        // the robot must then stay within this tolerance for a certain amount of time
        // if the robot exits the error tolerance, the timer restarts
        if(Math.abs(error) < tolerance){
            timer.startTime();
            if(Math.abs(error) > tolerance){
                timer.reset();
            }
        }

         return Math.abs(error) < tolerance && timer.milliseconds() > 30000;
    }

    @Override
    public void end(boolean interupted) {
       driveTrain.drive(0,0,0,0);
    }
}
