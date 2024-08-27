package org.firstinspires.ftc.teamcode.systems.crash_detection;

import android.graphics.Canvas;
import android.graphics.Point;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class TagProcessor implements VisionProcessor {

    public Scalar lower = new Scalar(0,0,0);
    public Scalar upper = new Scalar(0,0,0);

    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat unmaskedInputMat = new Mat();

    private int pixelthreshold = 100;
    public boolean objectDetected = false;

    private Point topLeft = new Point(10,0), bottomRight = new Point(40, 20);


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, upper, lower, binaryMat);

        int coloredPixelCount = 0;
        for(int i = (int) topLeft.x; i <= bottomRight.x; i++){
            for(int j = (int) topLeft.y; j <= bottomRight.y; j++){
                if(binaryMat.get(i, j)[0] == 255) {
                    coloredPixelCount++;
                }
            }
        }
        if(coloredPixelCount > pixelthreshold){
            objectDetected = true;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    private void chooseQuadrent(){

    }

    private void crossHair(Point topLeft, Point bottomRight){
        double midpointX = (bottomRight.x - topLeft.x) / 2;
        double midpointY = (topLeft.y - bottomRight.y) / 2;
    }
}
