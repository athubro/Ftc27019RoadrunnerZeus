package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ElementDetector extends OpenCvPipeline {
    // Define the positions we want to check (adjust X and Y coordinates for the game)
    public enum Position { LEFT, CENTER, RIGHT }
    private Position finalPosition = Position.CENTER;

    // Define ROI (Region of Interest) bounding boxes
    // Rect(x, y, width, height)
    static final Rect LEFT_ROI = new Rect(20, 100, 50, 50);
    static final Rect CENTER_ROI = new Rect(140, 100, 50, 50);

    Mat mat = new Mat();
    Mat leftSquare = new Mat();
    Mat centerSquare = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV color space (great for color detection)
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Crop the regions we want to look at
        leftSquare = mat.submat(LEFT_ROI);
        centerSquare = mat.submat(CENTER_ROI);

        // Extract the Saturation channel (or Hue depending on your targets)
        // This is index 1 in HSV (0=H, 1=S, 2=V)
        Scalar leftAvg = Core.mean(leftSquare);
        Scalar centerAvg = Core.mean(centerSquare);

        double leftValue = leftAvg.val[1];
        double centerValue = centerAvg.val[1];

        // Clean up sub-matrices to prevent memory leaks
        leftSquare.release();
        centerSquare.release();

        // Threshold logic: find where the element is based on the highest saturation
        double threshold = 100; // Adjust based on your team's element color
        if (leftValue > threshold) {
            finalPosition = Position.LEFT;
        } else if (centerValue > threshold) {
            finalPosition = Position.CENTER;
        } else {
            finalPosition = Position.RIGHT;
        }

        // Optional: Draw rectangles on the camera view for debugging
        Imgproc.rectangle(input, LEFT_ROI, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, CENTER_ROI, new Scalar(0, 255, 0), 2);

        return input;
    }

    public Position getPosition() {
        return finalPosition;
    }
}