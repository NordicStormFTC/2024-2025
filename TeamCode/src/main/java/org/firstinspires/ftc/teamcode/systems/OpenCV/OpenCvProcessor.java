package org.firstinspires.ftc.teamcode.systems.OpenCV;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Point;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class OpenCvProcessor implements VisionProcessor {

    public String output = "BLAAAAA";

    public Scalar lower = new Scalar(0,0,0);
    public Scalar upper = new Scalar(0,0,0);

    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();
    private Mat unmaskedInputMat = new Mat();

    private int pixelthreshold = 100;

    public boolean objectDetected = false;

    private Point topLeft = new Point(10,0), bottomRight = new Point(40, 20);

    public String getOutput(){
        return output;
    }

//    public OpenCvProcessor(Point topleft, Point bottomRight, int pixelThreshold, Scalar upper, Scalar lower){
//        this.bottomRight = bottomRight;
//        this.topLeft = topleft;
//        this.lower = lower;
//        this.upper = upper;
//        this.pixelthreshold = pixelThreshold;
//    }
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

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.bottom * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.top * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.left * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.right * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity * 4);

//        canvas.drawRect(makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx), rectPaint);
//        canvas.drawRect(makeGraphicsRect(rectSide, scaleBmpPxToCanvasPx), rectPaint);
    }
}
