package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class StrafeForInches extends CommandBase {
    private DriveTrain driveTrain;
    private double rollingPos;
    private double odoOffset;
    private double target;
    private double error;
    private double targetAngle;

    // Its not a bug, its an undocumented feature :0
    public StrafeForInches(double target, DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        this.target = target;

    }

    @Override
    public void initialize() {
        odoOffset = driveTrain.frontLeft.getCurrentPosition();
    }

    @Override
    public void execute() {
        rollingPos = driveTrain.frontLeft.getCurrentPosition() - odoOffset;
        error = target - rollingPos;
        double velocity = driveTrain.imu.getAcceleration().xAccel;
        double xP = 0.08 * error;
        double xD = .008 * velocity;
        if(xP > 1){
            xP/=xP;
        }
        double xPower = xP - xD;
        driveTrain.drive(xPower,0 , 0, .8);
    }

    @Override
    public boolean isFinished() {

        return Math.abs(error) < 4;
    }

    @Override
    public void end(boolean interupted) {

        driveTrain.drive(0,0,0,0);
    }


}
