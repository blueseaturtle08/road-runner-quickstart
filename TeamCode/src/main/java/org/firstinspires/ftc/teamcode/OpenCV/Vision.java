package org.firstinspires.ftc.teamcode.OpenCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Vision extends OpenCvPipeline {

    Telemetry telemetry;
    private String detectedColor;
    private String leftOrRight;
    Mat mat = new Mat();
    Mat yellowmat = new Mat();
    static Rect Rect_right;
    static Rect Rect_center;
    static Rect Rect_left;
    private int location;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar yellowhighHSV = new Scalar(30, 255, 255);
        Scalar yellowlowHSV = new Scalar(20, 100, 100);
        Rect Rect_left = new Rect(
                new Point(60, 160),
                new Point(100, 110)
        );

        Rect Rect_center = new Rect(
                new Point(130, 160),
                new Point(180, 110)
        );

        Rect Rect_right = new Rect(
                new Point(220, 155),
                new Point(270, 105)
        );
        Scalar color = new Scalar(64, 64, 64);
        Imgproc.rectangle(mat, Rect_left, color, 2);
        Imgproc.rectangle(mat, Rect_center, color, 2);
        Imgproc.rectangle(mat, Rect_right, color, 2);

        Core.inRange(mat, yellowlowHSV, yellowhighHSV, yellowmat);
        Mat left = yellowmat.submat(Rect_left);
        Mat center = yellowmat.submat(Rect_center);
        Mat right = yellowmat.submat(Rect_right);

        double leftValue = Core.sumElems(left).val[0];
        double centerValue = Core.sumElems(center).val[0];
        double rightValue = Core.sumElems(right).val[0];
        if (leftValue > centerValue || leftValue > rightValue) {
            telemetry.addData("Duck", "Left");
        }
        if (centerValue > leftValue || centerValue > rightValue) {
            telemetry.addData("Duck", "Middle");
        }
        if (rightValue > centerValue || rightValue > leftValue) {
            telemetry.addData("Duck", "Right");
        }

        telemetry.addData("test", "test");
        telemetry.addData("leftValue", leftValue + "");
        telemetry.addData("centerValue", leftValue + "");
        telemetry.addData("rightValue", leftValue + "");
        telemetry.update();
        return mat;
    }
}
