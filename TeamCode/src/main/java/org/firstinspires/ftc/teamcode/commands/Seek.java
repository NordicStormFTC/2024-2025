package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.systems.OpenCV.OpenCvProcessor;

public class Seek extends CommandBase {

    DriveTrain driveTrain;
    OpenCvProcessor processor;

    private double adjustedX;
    private double adjustedY;

    private double xError;
    private double yError;

    private double xErrorTolerance;

    private ElapsedTime timer = new ElapsedTime();

    public Seek(DriveTrain driveTrain, OpenCvProcessor processor){
        this.driveTrain = driveTrain;
        this.processor = processor;
        timer.reset();
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        xError = processor.averageX - adjustedX;
        yError = processor.averageY - adjustedY;

        double rP = 0.008 * xError;
        driveTrain.drive(0,0, rP,1);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(xError) < xErrorTolerance){
            timer.startTime();
            if(Math.abs(xError) > xErrorTolerance){
                timer.reset();
            }
        }

        return Math.abs(xError) < 2 && timer.milliseconds() > 4000;
    }

    @Override
    public void end(boolean isInterupted){
        driveTrain.drive(0,0,0,0);
    }

    public void zeroVision(){

    }

}
