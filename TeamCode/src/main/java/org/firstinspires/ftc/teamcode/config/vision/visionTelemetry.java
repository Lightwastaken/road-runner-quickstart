package org.firstinspires.ftc.teamcode.config.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class visionTelemetry extends OpenCvPipeline{
    // Lens intrinsics
    // UNITS ARE PIXELS
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy =  242.502;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest;
    Telemetry telemetry;
    String robotParksAt;

    AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

    public visionTelemetry(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input) {
        try {
            for (AprilTagDetection tag : currentDetections) {
                tagOfInterest = tag;
                tagOfInterest.id = tag.id;
            }
            switch (tagOfInterest.id) {
                case 2: //RANDOMIZATION = 2, 5
                    robotParksAt = "MIDDLE";
                    break;
                case 3: //RANDOMIZATION = 3, 6
                    robotParksAt = "RIGHT";
                    break;
                default:
                    robotParksAt = "LEFT";
            }

            telemetry.addData("Tag ID: ", tagOfInterest.id);
            telemetry.addData("Robot parking location: ", robotParksAt);

        } catch (Exception e) {
            telemetry.addLine("Tag not found");
        }

        telemetry.update();

        return input;
    }

    public int getID() {
        return tagOfInterest.id;
    }
}