package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.systems.Heading;
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
        DriveTrain driveTrain = new DriveTrain(hardwareMap, this);
        Heading startAngle = new Heading(0);
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


       driveTrain.initMotorEncoders();
       telemetry.addData("done", true);
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("front left",driveTrain.frontLeft.getCurrentPosition());
            telemetry.addData("back left",driveTrain.backLeft.getCurrentPosition());
            telemetry.addData("front right",driveTrain.frontRight.getCurrentPosition());
            telemetry.addData("back right",driveTrain.backRight.getCurrentPosition());



           driveTrain.driveForDistanceX(.5, 1000);

//   if( driveTrain.backRight.getCurrentPosition() < 1000 && opModeIsActive()){
//       driveTrain.backRight.setPower(1);
//        driveTrain.frontRight.setPower(1);
//         driveTrain.backLeft.setPower(1);
//           driveTrain.backRight.setPower(.5);
//
//    } else if(driveTrain.backRight.getCurrentPosition() > 1000 && opModeIsActive()){
//
//       driveTrain.backRight.setPower(0);
//   }
//
//            driveTrain.backRight.setPower(0);
//
//            driveTrain.backLeft.setPower(0);
//            driveTrain.frontRight.setPower(0);

            telemetry.update();
        }
    }
}
