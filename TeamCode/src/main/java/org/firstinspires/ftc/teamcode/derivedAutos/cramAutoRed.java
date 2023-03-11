/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.derivedAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.config.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="cramautoR ", group="Pushbot")
public class cramAutoRed extends LinearOpMode {
    public static Pose2d preloadEnd;
    public static Pose2d cycleEnd;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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


    @Override
    public void runOpMode()
    {
        AprilTagDetection tagOfInterest = null;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RobotHardware robot = new RobotHardware(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        Pose2d start = new Pose2d(36, 60, Math.toRadians(0));
        drive.setPoseEstimate(start);

        telemetry.setMsTransmissionInterval(50);
        robot.initHW();
        /*

         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT||tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        TrajectorySequence preloadDeliver = drive.trajectorySequenceBuilder(start)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.claw.setPosition(1);
                })
                .back(48)
                // preload [ not implmented]
                .waitSeconds(0.5)
                .turn(Math.toRadians(45))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.AutoPIDControl(RobotHardware.MID_JUNC, "M"); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.claw.setPosition(0.9); })
                .waitSeconds(0.3)
                .back(4)
                .turn(Math.toRadians(-135))
                .forward(25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.AutoPIDControl(800, "CS1"); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.claw.setPosition(1); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.AutoPIDControl(RobotHardware.MID_JUNC, "M"); })
                .waitSeconds(0.2)
                .back(25)
                .turn(Math.toRadians(135))
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.claw.setPosition(0.9); })
                .waitSeconds(0.3)
                .back(4)
                .turn(Math.toRadians(-45))
                .build();

        preloadEnd = preloadDeliver.end();


        TrajectorySequence leftTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .strafeLeft(19)
                .build();

        TrajectorySequence middleTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .back(0.25)
                .strafeRight(5)
                .build();

        TrajectorySequence rightTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .back(0.25)
                .strafeRight(30)
                .build();


        //TRAJECTORY FOLLOWED
        drive.followTrajectorySequence(preloadDeliver);
        if (tagOfInterest == null || tagOfInterest.id == LEFT) { //LEFT parking: ID #1
            drive.followTrajectorySequence(leftTOI);
        } else if (tagOfInterest.id == MIDDLE) { //MIDDLE parking: ID #2
            drive.followTrajectorySequence(middleTOI);
        } else { //RIGHT parking; ID #3
            drive.followTrajectorySequence(rightTOI);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void cycles(int numCycles, SampleMecanumDrive drive, RobotHardware robot) {
        for (int i = 0; i < numCycles; i++) {
            TrajectorySequence cycle = drive.trajectorySequenceBuilder(preloadEnd)
                    .UNSTABLE_addDisplacementMarkerOffset(12 - 2 * i, () -> {
                        robot.claw.setPosition(1);
                        robot.lift(-0.01);
                    })
                    .lineToSplineHeading(new Pose2d(57, -11.5, Math.toRadians(0)))
                    .addDisplacementMarker(() -> {
                        robot.lift(0);
                    })
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        robot.claw.setPosition(0);
                    })
                    .waitSeconds(0.3)
                    .addTemporalMarker(() -> {
                        robot.lift(0.1);
                    })
                    .waitSeconds(1)
                    .lineToSplineHeading(new Pose2d(24, -12, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        robot.lift(-0.01);
                    })
                    .waitSeconds(1)
                    .addTemporalMarker(() -> {
                        robot.claw.setPosition(1);
                        robot.lift(0.05);
                    })
                    .waitSeconds(0.5)
                    .build();
            cycleEnd = cycle.end();
        }
    }
}