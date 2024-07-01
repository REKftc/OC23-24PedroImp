package overcharged.test;

import static overcharged.config.RobotConstants.TAG_A;

import com.qualcomm.robotcore.util.RobotLog;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import overcharged.components.propLocation;

public class HSVPipeline extends OpenCvPipeline {
    double x1 = 0.45;//.415//0.44;//0.46;//0.35;//0.42;
    double x2 = 0.5;//.475//0.51;//0.4;//0.54;
    double y1 = 0.65;//0.55;
    double y2 = 0.55;//0.29;
    double rightX1 = 0.945;//0.86;//0.9;//0.85;//0.02;
    double rightX2 = 0.99;//0.95;//0.9;//0.15;
    double rightY1 = 0.72;//0.8;//0.61;
    double rightY2 = 0.62;//0.35;

    //blues
    double blueX1 = 0.67;//0.71;//0.74;//0.64;//0.69;
    double blueX2 = 0.72;//0.76;//0.79;//0.69;//0.74;
    double blueY1 = 0.68;//0.72;
    double blueY2 = 0.58;//0.62;
    double blueLeftX1 = 0.25;//0.29;//0.24;//0.27;
    double blueLeftX2 = 0.3;//0.34;//0.29;//0.32;
    double blueLeftY1 = 0.73;
    double blueLeftY2 = 0.63;
    Scalar averageColor, rightAverageColor, blueAverageColor, blueLeftAverageColor;
    Scalar lowerRed1 = new Scalar(0, 100, 100);
    Scalar upperRed1 = new Scalar(15, 255, 255);
    Scalar lowerRed2 = new Scalar(113, 100, 70);
    Scalar upperRed2 = new Scalar(180, 255, 255);
    Scalar lowerBlue = new Scalar(0, 140, 33);
    Scalar upperBlue = new Scalar(37, 255, 255);
    boolean midColor = false;
    boolean rightColor = false;
    boolean blueMidColor = false;
    boolean blueLeftColor = false;
    boolean closeBlueMid = false;
    boolean closeBlueRight = false;
    boolean closeRedMid = false;
    boolean closeRedLeft = false;
    private propLocation location = propLocation.Middle;

    @Override
    public Mat processFrame(Mat input) {
        //red mid
        Point tl = new Point(x1 * input.cols(), y1 * input.rows());
        Point br = new Point(x2 * input.cols(), y2 * input.rows());
        Mat rawImage = input.submat((int) (y2 * input.rows()), (int) (y1 * input.rows()), (int) (x1 * input.cols()), (int) (x2 * input.cols()));
        //red right box
        Point rightTl = new Point(rightX1 * input.cols(), rightY1 * input.rows());
        Point rightBr = new Point(rightX2 * input.cols(), rightY2 * input.rows());
        Mat rightRawImage = input.submat((int) (rightY2 * input.rows()), (int) (rightY1 * input.rows()), (int) (rightX1 * input.cols()), (int) (rightX2 * input.cols()));
        //blue mid box
        Point blueTl = new Point(blueX1 * input.cols(), blueY1 * input.rows());
        Point blueBr = new Point(blueX2 * input.cols(), blueY2 * input.rows());
        Mat blueRawImage = input.submat((int) (blueY2 * input.rows()), (int) (blueY1 * input.rows()), (int) (blueX1 * input.cols()), (int) (blueX2 * input.cols()));
        //blue left box
        Point blueLeftTl = new Point(blueLeftX1 * input.cols(), blueLeftY1 * input.rows());
        Point blueLeftBr = new Point(blueLeftX2 * input.cols(), blueLeftY2 * input.rows());
        Mat blueLeftRawImage = input.submat((int) (blueLeftY2 * input.rows()), (int) (blueLeftY1 * input.rows()), (int) (blueLeftX1 * input.cols()), (int) (blueLeftX2 * input.cols()));

        // convert the cropped region to HSV color space
        //mid red
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(rawImage, hsvImage, Imgproc.COLOR_BGR2HSV);
        //right red
        Mat rightHsvImage = new Mat();
        Imgproc.cvtColor(rightRawImage, rightHsvImage, Imgproc.COLOR_BGR2HSV);
        //mid blue
        Mat blueHsvImage = new Mat();
        Imgproc.cvtColor(blueRawImage, blueHsvImage, Imgproc.COLOR_BGR2HSV);
        //left blue
        Mat blueLeftHsvImage = new Mat();
        Imgproc.cvtColor(blueLeftRawImage, blueLeftHsvImage, Imgproc.COLOR_BGR2HSV);
        // Calculate the average color within the ROI
        try {
            averageColor = Core.mean(hsvImage);
        } catch (Exception e) {
            e.printStackTrace();
        }

        rightAverageColor = Core.mean(rightHsvImage);
        blueAverageColor = Core.mean(blueHsvImage);
        blueLeftAverageColor = Core.mean(blueLeftHsvImage);

        midColor =  isColorInRange(averageColor, lowerRed1, upperRed1) || isColorInRange(averageColor, lowerRed2, upperRed2);
        rightColor = isColorInRange(rightAverageColor, lowerRed1, upperRed1) || isColorInRange(rightAverageColor, lowerRed2, upperRed2);
        blueMidColor = isColorInRange(blueAverageColor, lowerBlue, upperBlue);
        blueLeftColor = isColorInRange(blueLeftAverageColor, lowerBlue, upperBlue);

        closeBlueMid = isColorInRange(averageColor, lowerBlue, upperBlue);
        closeBlueRight = isColorInRange(rightAverageColor, lowerBlue, upperBlue);
        closeRedMid =  isColorInRange(blueAverageColor, lowerRed1, upperRed1) || isColorInRange(blueAverageColor, lowerRed2, upperRed2);
        closeRedLeft =  isColorInRange(blueLeftAverageColor, lowerRed1, upperRed1) || isColorInRange(blueLeftAverageColor, lowerRed2, upperRed2);

        // Release resources
        hsvImage.release();
        rightHsvImage.release();
        // check side
        /*
        if(midColor)
            location = propLocation.Middle;
        else if(rightColor)
            location = propLocation.Right;
        else if(!midColor&&!rightColor){
            location = propLocation.Left;
        }*/
        // Draw the rectangle on the original input
        Imgproc.rectangle(input, tl, br, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, rightTl, rightBr, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, blueTl, blueBr, new Scalar(0, 0, 255), 2);
        Imgproc.rectangle(input, blueLeftTl, blueLeftBr, new Scalar(0, 0, 255), 2);
        RobotLog.ii(TAG_A, "color: p" + midColor);
        RobotLog.ii(TAG_A, "avg color p : " + averageColor);
        //RobotLog.ii(TAG_A, "location pipeline: " + location);
        RobotLog.ii(TAG_A, "right color: p" + rightColor);
        RobotLog.ii(TAG_A, "right avg color p : " + rightAverageColor);
        return input;
    }

    private boolean isColorInRange(Scalar color, Scalar lowerBound, Scalar upperBound) {
        return color.val[0] >= lowerBound.val[0] && color.val[0] <= upperBound.val[0]
                && color.val[1] >= lowerBound.val[1] && color.val[1] <= upperBound.val[1]
                && color.val[2] >= lowerBound.val[2] && color.val[2] <= upperBound.val[2];
    }

    public Scalar getAverageColor() {
        return averageColor;
    }

    public Scalar getRightAverageColor() {
        return rightAverageColor;
    }
    public Scalar getBlueAverageColor() {
        return blueAverageColor;
    }
    public Scalar getBlueLeftAverageColor() {
        return blueLeftAverageColor;
    }

    public propLocation getLocation(boolean red, boolean close) {
        if (red && !close) {
            //      midColor = isColorInRange(averageColor, lowerRed1, upperRed1) || isColorInRange(averageColor, lowerRed2, upperRed2);
            //    rightColor = isColorInRange(rightAverageColor, lowerRed1, upperRed1) || isColorInRange(rightAverageColor, lowerRed2, upperRed2);
            if (midColor)
                location = propLocation.Middle;
            else if (rightColor)
                location = propLocation.Right;
            else if (!midColor && !rightColor) {
                location = propLocation.Left;
            }
        }
        if (red && close) {
            if (closeRedMid)
                location = propLocation.Middle;
            else if (closeRedLeft)
                location = propLocation.Left;
            else if (!closeRedMid && !closeRedLeft) {
                location = propLocation.Right;
            }
        }
        //blue close
        if (!red && close) {
            if (closeBlueMid)
                location = propLocation.Middle;
            else if (closeBlueRight)
                location = propLocation.Right;
            else if (!closeBlueRight && !closeBlueMid) {
                location = propLocation.Left;
            }
        }
        if (!red&&!close) {
            // blueMidColor = isColorInRange(blueAverageColor, lowerBlue, upperBlue);
            //blueLeftColor = isColorInRange(blueLeftAverageColor, lowerBlue, upperBlue);
            if (blueMidColor)
                location = propLocation.Middle;
            else if (blueLeftColor)
                location = propLocation.Left;
            else if (!blueMidColor && !blueLeftColor) {
                location = propLocation.Right;
            }
        }
        return location;
    }
}