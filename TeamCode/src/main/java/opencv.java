

import android.sax.TextElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


import java.io.CharArrayReader;
import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@TeleOp(name = "opencv")public class opencv extends LinearOpMode {

    private OpenCvCamera camera;
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;

    private  int pathState = 0;
    private  Follower follower = null;


    PathChain line1 = null;

    PathChain line2 = null;

    Servo horizontalClawSwivel = null;
    DcMotor horizontalLiftLeftMotor = null;
    DcMotor horizontalLiftRightMotor = null;
    Servo horizontalSwivelRight = null;
    Servo horizontalSwivelLeft = null;

    Servo horizontalLiftClaw = null;

    YellowBlobDetectionPipeline pipeline = new YellowBlobDetectionPipeline();

    private void initOpenCv() {
        int camMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), camMonitorViewId);


        camera.openCameraDevice();
        camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);

        horizontalSwivelLeft.setDirection(Servo.Direction.REVERSE);
    }

    @Override public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);


        follower.drawOnDashBoard();
        follower.setStartingPose(new Pose(8, 80));
        line1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new com.pedropathing.pathgen.Point(8.000, 80.000, com.pedropathing.pathgen.Point.CARTESIAN),
                                new com.pedropathing.pathgen.Point(37.587, 79.886, com.pedropathing.pathgen.Point.CARTESIAN)
                        )
                )
                //.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                .setTangentHeadingInterpolation()
                .build();

        line2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new com.pedropathing.pathgen.Point(37.587, 80.000, com.pedropathing.pathgen.Point.CARTESIAN),
                                new com.pedropathing.pathgen.Point(37.587, 38, com.pedropathing.pathgen.Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        horizontalClawSwivel = hardwareMap.get(Servo.class, "horizontalClawSwivel");
        horizontalLiftLeftMotor = hardwareMap.dcMotor.get("horizontalLiftLeft");
        horizontalLiftRightMotor = hardwareMap.dcMotor.get("horizontalLiftRight");
        horizontalSwivelRight = hardwareMap.get(Servo.class, "swivelRight");
        horizontalSwivelLeft = hardwareMap.get(Servo.class, "swivelLeft");

        horizontalLiftClaw = hardwareMap.get(Servo.class, "horizontalLiftClaw");

        horizontalLiftClaw.setPosition(0.5);


        horizontalSwivelLeft.setPosition(degToServoPosSwivel(130));
        horizontalSwivelRight.setPosition(degToServoPosSwivel(130));


        initOpenCv();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        FtcDashboard.getInstance().startCameraStream(camera, 30);

        camera.setPipeline(pipeline);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //pathUpdate();
//                follower.update();

                Point p = pipeline.getCenter();
                if (p != null) {
                    follower.turn(pixelToCorrectionDeg(p.y, p.x), true);
                    follower.setMaxPower(0.7);
                    follower.update();
                    telemetry.addData("deg", pixelToCorrectionDeg(p.y, p.x));
                    telemetry.update();
                }
            }
        }
    }

    private void pathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1, true);

                pathState = 1;

                break;

            case 1:

                if (follower.isBusy()){
                    break;
                }

                Point p = pipeline.getCenter();
                if (p != null && horizontalLiftRightMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    horizontalLiftRightMotor.setTargetPosition((int)pixelToEncoderPos(p.x));
                    horizontalLiftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horizontalLiftRightMotor.setPower(0.5);
                    horizontalLiftClaw.setPosition(0.5);
                }

                if (Math.abs(horizontalLiftRightMotor.getCurrentPosition() - horizontalLiftRightMotor.getTargetPosition()) < 10) {
                    horizontalSwivelLeft.setPosition(degToServoPosSwivel(0));
                    horizontalSwivelRight.setPosition(degToServoPosSwivel(0));
                    horizontalClawSwivel.setPosition(degToServoPosClawSwivel(pipeline.getBlockAngle() - 90));


                    sleep(2000);
                    horizontalLiftClaw.setPosition(1);
                    sleep(1000);

                    pathState = 1;
                    follower.followPath(line2, true);
                    pathState = 2;
                }
        }
    }

    public static double degToServoPosSwivel(double deg) {
        return (270 - deg) / 600 + 0.66666666667;
    }


    public static double degToServoPosClawSwivel(double deg) {
        if (deg == 0 || Double.isNaN(deg)) {
            return 0;
        }

        deg = Math.abs(deg);

        deg *= 0.666667;

        return (deg / 180) - Math.floor(deg / 180);
    }

    public double pixelToEncoderPos(double y) {
        y = y;
        double scalar = 370.0f / 640; //480 - 110

        return y * scalar + 30;
    }

    public double pixelToCorrectionDeg(double x, double y) {
        double pos_x = x / 480 * 20.5 + 5; // -5, +5 or 0?
        double pos_y = y / 640 * 15.5;

        double b = (10.25 - pos_x);
        double c = Math.sqrt(Math.pow(10.25 - pos_x, 2) + Math.pow(pos_y, 2));

        return Math.asin(b/c);
    }

    //y 15.5 x 20.5


    class YellowBlobDetectionPipeline extends OpenCvPipeline {

        double cX = 0;
        double cY = 0;
        double width = 0;

        public final double objectWidthInRealWorldUnits = 3.5;  // Replace with the actual width of the object in real-world units
        public final double focalLength = 728;  // Replace with the focal length of the camera in pixels

        Point[] rectVertices = new Point[4];

        Point center = null;

        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                center = rect.center;

                rect.points(rectVertices);

                // Draw the rotated rectangle
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, rectVertices[i], rectVertices[(i + 1) % 4], new Scalar(0, 255, 0), 4);
                }

                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                Imgproc.putText(input, Double.toString(getBlockAngle()), new Point(cX - 30, cY+50), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 0, 0), 2);

                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

            Scalar lowerYellow = new Scalar(20, 60, 60);
            Scalar upperYellow = new Scalar(80, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);



            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

        public double getBlockAngle() {
            if (rectVertices[0] == null || rectVertices[1] == null || rectVertices[2] == null
                    || rectVertices[3] == null) {
                return 0;
            }

            double offset = 0;

            double c = Math.sqrt(Math.pow(rectVertices[0].x - rectVertices[1].x, 2) + Math.pow(rectVertices[0].y - rectVertices[1].y, 2));
            double b = Math.abs(rectVertices[0].y - rectVertices[1].y);

            if (Math.sqrt(Math.pow(rectVertices[1].x - rectVertices[2].x, 2) + Math.pow(rectVertices[1].y - rectVertices[2].y, 2)) > c) {
                offset = 90;
            }

            if (rectVertices[3].y > rectVertices[1].y) {
                offset = 180 - offset;
            }


            return Math.asin(b / c) * 57.3 + offset;
        }

        public Point getCenter() {
            return center;
        }

    }
}