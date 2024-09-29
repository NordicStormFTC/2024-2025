package org.firstinspires.ftc.teamcode.commands;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.systems.DriveTrain;

public class Pivot implements Command {


    private double degrees;
    private DriveTrain driveTrain;
    double error = degrees;

    public Pivot(double degrees, DriveTrain driveTrain){
        this.degrees = degrees;
        this.driveTrain = driveTrain;

    }
    @Override
    public void initialize() {
        double referenceAngle = driveTrain.getAngle();
        degrees = Math.toDegrees(angleWrap(1));
    }

    @Override
    public void execute() {
//        double error = driveTrain.getAngle() - refer;
    }

    @Override
    public boolean isDone() {
//        if(driveTrain.getAngle())

        return false;
    }

    @Override
    public void end() {

    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }
}
