package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.systems.OpenCV.OpenCvProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous

public class Auto2024 extends LinearOpMode {

    // this resets the command scheduler and wipes all slated commands
    public void reset(){
        CommandScheduler.getInstance().reset();
    }

    // this runs the scheduler and executes all the scheduled commands
    public void run(){
        CommandScheduler.getInstance().run();

    }

    // schedules one singular command/sequential command/ parallel command
    public void schedule(Command... commands){
        CommandScheduler.getInstance().schedule(commands);
    }

   private VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);

        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), driveTrain.openCvProcessor);

        if(driveTrain.openCvProcessor.objectDetected){
            telemetry.addData("Object detected", true);
        } else {
            telemetry.addData("no", true);

        }
        reset();

        telemetry.addData("pixels", driveTrain.openCvProcessor.superstupid);
        schedule(new AutoCommandStack(driveTrain));
        telemetry.addData("done", true);
        telemetry.update();
        waitForStart();

        while( opModeIsActive()){
            telemetry.addData("avx", driveTrain.openCvProcessor.averageY);
            telemetry.addData("pixels", driveTrain.openCvProcessor.stupid);
            telemetry.update();
            run();
        }

        reset();

    }

}
