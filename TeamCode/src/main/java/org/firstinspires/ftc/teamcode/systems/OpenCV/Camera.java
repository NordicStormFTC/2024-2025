package org.firstinspires.ftc.teamcode.systems.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera {
//    private OpenCvWebcam webcam;
//    private HardwareMap hardwareMap;
//    private VisionProcessor pipeline;
//
//    public Camera(HardwareMap hardwareMap) {
//        pipeline = new OpenCvPipeline();
//        this.hardwareMap = hardwareMap;
//        int cameraMoniterViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMoniterViewId", "id", hardwareMap.appContext.getPackageName());
//        //configures the camera in the hardware
//
//        //gets "camera" from control hub and replaces it with what is in your control hub
//
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"));
//        webcam.setPipeline(pipeline);
//        webcam.setMillisecondsPermissionTimeout(2500);
//
//        //Streaming frames
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//    }
//        public String getOutput(){
//            return pipeline.getOutput();
//        }
//
//        public void stop() {
//            webcam.stopStreaming();
//        }
//    }
}
