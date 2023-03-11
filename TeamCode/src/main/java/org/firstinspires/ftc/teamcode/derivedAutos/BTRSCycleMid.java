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
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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
import java.util.concurrent.atomic.AtomicInteger;


@Autonomous(name="Blue terminal red substation", group="Pushbot")
public class BTRSCycleMid extends LinearOpMode {
    public static Pose2d preloadEnd;
    public static Pose2d cycleEnd;
    int[] coneStack = {200, 160, 120, 80, 40};

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

        Pose2d start = new Pose2d(33, -60, Math.toRadians(-90));
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
                    robot.claw.setPosition(0.8);
                })
                .strafeLeft(2)
                .lineToLinearHeading(new Pose2d(34,-12,Math.toRadians(-135)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.AutoPIDControl(RobotHardware.MID_JUNC, "M"); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.claw.setPosition(0);})
                .waitSeconds(0.4)
                // preload ^
                .build();

        preloadEnd = preloadDeliver.end();


        TrajectorySequence leftTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .strafeLeft(19)
                .build();

        TrajectorySequence middleTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .back(0.25)
                .strafeRight(13)
                .build();

        TrajectorySequence rightTOI = drive.trajectorySequenceBuilder(preloadEnd)
                .back(0.25)
                .strafeRight(39)
                .build();


        //TRAJECTORY FOLLOWED
        drive.followTrajectorySequence(preloadDeliver);
        cycles(1,  drive, robot);
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
        AtomicInteger stack = new AtomicInteger();
        TrajectorySequence cycles = drive.trajectorySequenceBuilder(preloadEnd)
                .lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(0)))
                // goes to cone stack ^
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.PIDControl(coneStack[stack.intValue()], robot.getLiftAvg()); })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.claw.setPosition(0.8); })
                // picks up stuff
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.AutoPIDControl(RobotHardware.MID_JUNC, "M"); })
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(34,-12,Math.toRadians(-135)))
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> { robot.claw.setPosition(0); })
                .build();

        for (int i = 0; i < numCycles; i++) {
            drive.followTrajectorySequence(cycles);
            stack.getAndIncrement();
        }

        cycleEnd = cycles.end();
    }
}