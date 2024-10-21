package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.Pivot;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;
//import org.opencv.core.Size;

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
//    private VisionPortal portal;
//    private OpenCvProcessor processor = new OpenCvProcessor();

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
// all this stuff works but the hardware isnt set up rn sooo i justed commented it out while testing commands
//        portal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
//                .addProcessor(processor)
//                .setCameraResolution(new Size(640, 480))
//                .setCamera(BuiltinCameraDirection.BACK)
//                .enableLiveView(true)
//                .build();
//        if(processor.objectDetected){
//            telemetry.addData("Object detected", true);
//        }
        reset();
        schedule(new AutoCommandStack(driveTrain, this));

        telemetry.addData("done", true);
        telemetry.update();
        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            run();
        }
        reset();

    }

}
