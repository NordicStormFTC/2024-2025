package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.systems.OpenCV.OpenCvProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.opencv.core.Size;

@Autonomous

public class Auto2024 extends LinearOpMode {

    private VisionPortal portal;
    private OpenCvProcessor processor = new OpenCvProcessor();

//    private LinearOpMode currentOpMode = this;

    @Override
    public void runOpMode() throws InterruptedException {
//        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)
                .enableLiveView(true)
                .build();
        if(processor.objectDetected){
            telemetry.addData("Object detected", true);
        }

       telemetry.addData("done", true);
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

//           driveTrain.driveForDistanceY(.5, 1000);
            telemetry.update();
        }

    }


}
