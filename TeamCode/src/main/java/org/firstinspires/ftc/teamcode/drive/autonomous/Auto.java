package org.firstinspires.ftc.teamcode.drive.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotHardware;
import org.firstinspires.ftc.teamcode.config.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;
import java.util.Arrays;

/*
AUTO NAMING CONVENTION:
1) first letter of alliance/side: _T_S
   EX: BLUE TERMINAL BLUE SUBSTATION: BTBS
   EX: RED TERMINAL RED SUBSTATION: RTBS
2) cycle or park?
    if CYCLE: add "Cycle"
    if PARK: add "Park"
3) which pole to cycle to
    high junction closest to cone stack: "CenterHigh"
    mid junction: "Mid"
    high junction on the line that separates quadrant of us and our alliance partner: "FarHigh"
    NOTE: skip if not cycling

EX: BTBSCycleCenterHigh = blue alliance, robot is next to blue terminal. robot will cycle to the high junction next to the cone stack

change CYCLE_COUNT variable to change # of cycles
*/

public abstract class Auto extends RobotHardware {
    //variable things
    private final boolean DIFF_COLOR_TERMINAL = modeNameContains("BTRS") || modeNameContains("RTBS");
    private final boolean SAME_COLOR_TERMINAL = !DIFF_COLOR_TERMINAL;
    private final boolean CYCLE = modeNameContains("Cycle");
    private final boolean PARK_ONLY = !CYCLE;
    private final boolean CYCLE_HIGH_CLOSE = modeNameContains("CenterHigh") && CYCLE;
    private final boolean CYCLE_MID = modeNameContains("Mid") && CYCLE;
    private final boolean CYCLE_HIGH_FAR = modeNameContains("FarHigh") && CYCLE;
    private final boolean RED_ALLIANCE_LEFT = modeNameContains("RS") && SAME_COLOR_TERMINAL;
    private final boolean RED_ALLIANCE_RIGHT = !RED_ALLIANCE_LEFT;
    private final boolean BLUE_ALLIANCE_RIGHT = modeNameContains("BS") && SAME_COLOR_TERMINAL;
    private final boolean BLUE_ALLIANCE_LEFT = !BLUE_ALLIANCE_RIGHT;
    private final int CYCLE_COUNT = (CYCLE) ? 5 : 0;

    //FIRST QUADRANT coordinates aka BLUE SUB, RED TERMINAL

    //STARTING POSE
    private Pose2d startPose;
    //AUTO STACK POSE
    private Pose2d stackPose;
    //HIGH POLE CLOSEST TO STARTER STACK
    private Pose2d centerHighPose;
    //MID POLE
    private Pose2d midPose;
    //HIGH POLE BETWEEN US + ALLIANCE PARTNER
    private Pose2d farHighPose;
    //LEFT PARKING POSITION
    private Pose2d leftTOI;
    //MIDDLE PARKING POSITION
    private Pose2d middleTOI;
    //RIGHT PARKING POSITION
    private Pose2d rightTOI;

    ArrayList<Pose2d> poses = new ArrayList<Pose2d>(Arrays.asList(startPose, stackPose, centerHighPose, midPose, farHighPose, leftTOI, middleTOI, rightTOI));

    Pose2d cycleEnd;
    Pose2d preloadEnd;

    //vision
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy =  242.502;
    double tagsize = 0.166;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest;

    //CONSTRUCTOR
    public Auto() {
        //QUADRANT 1 poses
        startPose = new Pose2d(34.0, 60.0, Math.toRadians(0));
        stackPose = new Pose2d(57.0, 12.0, Math.toRadians(0));
        centerHighPose = new Pose2d(12.0, 36.0, Math.toRadians(-135));
        midPose = new Pose2d(36.0, 36.0, Math.toRadians(-135));
        farHighPose = new Pose2d(12.0, 12.0, Math.toRadians(-135));
        leftTOI = new Pose2d(64.0, 36.0, Math.toRadians(180));
        middleTOI = new Pose2d(36.0, 36.0, Math.toRadians(180));
        rightTOI = new Pose2d(12.0, 36.0, Math.toRadians(90));

        if (BLUE_ALLIANCE_RIGHT) { //BLUE SUB, BLUE TERMINAL. QUADRANT 2
            for (int i = 0; i < poses.size(); i++) {
                poses.set(i, reflectOverY(poses.get(i)));
            }
        } else if (RED_ALLIANCE_LEFT) { //RED SUB, RED TERMINAL. QUADRANT 3
            for (int i = 0; i < poses.size(); i++) {
                poses.set(i, reflectOverXY(poses.get(i)));
            }
        } else if (RED_ALLIANCE_RIGHT) { //RED SUB, BLUE TERMINAL. QUADRANT 4
            for (int i = 0; i < poses.size(); i++) {
                poses.set(i, reflectOverX(poses.get(i)));
            }
        }

        drive.setPoseEstimate(startPose);
    }

    //AUTO PATH
    public void runOpMode() {
        initialize();
        preload(); //Cycle auto only
        cycle(); //Cycle auto only
        park();
    }


    public void initialize() {
        init();
        telemetry.addLine("Hardware initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) {}
        });

        while (!linearOpMode.isStarted() && !linearOpMode.isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            String parkingLocation = "Tag not found";
//            for(AprilTagDetection tag : currentDetections) {
//                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                    tagOfInterest = tag;
//                    parkingLocation = tag.toString();
//                    break;
//                }
//            }

            parkingLocation = currentDetections.get(0).toString();

            //telemetry.addData("\nTag of Interest", tagOfInterest.id);
            telemetry.addData("Parking location", parkingLocation);
            telemetry.update();
        }
    }

    public void preload() {
        if (CYCLE) {
            TrajectorySequence preload;
            if (CYCLE_HIGH_CLOSE) {
                preload = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(centerHighPose)
                        .waitSeconds(0.5)
                        .build();
            } else if (CYCLE_MID) {
                preload = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(midPose)
                        .waitSeconds(0.5)
                        .build();
            } else {
                Pose2d temp = new Pose2d(0, 48, 0);
                Pose2d tempPose = (RED_ALLIANCE_LEFT || RED_ALLIANCE_RIGHT) ? farHighPose.minus(temp) : farHighPose.plus(temp);
                preload = drive.trajectorySequenceBuilder(startPose)
                        .splineToConstantHeading(tempPose.vec(), tempPose.getHeading())
                        .lineToLinearHeading(farHighPose)
                        .waitSeconds(0.5)
                        .build();
            }

            drive.followTrajectorySequence(preload);
            preloadEnd = drive.getPoseEstimate();
        } else {
            preloadEnd = startPose;
        }
    }

    public void cycle() {
        if (CYCLE) {
            TrajectorySequence cycle;
            if (CYCLE_HIGH_CLOSE) {
                cycle = drive.trajectorySequenceBuilder(preloadEnd)
                        .lineToLinearHeading(stackPose)
                        .waitSeconds(0.4)
                        .lineToLinearHeading(preloadEnd)
                        .build();
            } else if (CYCLE_MID) {
                Pose2d temp = new Pose2d(0, 24, Math.toRadians(-90));
                Pose2d tempPose = (RED_ALLIANCE_LEFT || RED_ALLIANCE_RIGHT) ? preloadEnd.plus(temp) : preloadEnd.minus(temp);
                cycle = drive.trajectorySequenceBuilder(preloadEnd)
                        .lineToLinearHeading(tempPose)
                        .lineToLinearHeading(stackPose)
                        .waitSeconds(0.4)
                        .lineToLinearHeading(tempPose)
                        .build();
            } else {
                Pose2d temp = new Pose2d(0, 24, Math.toRadians(-90));
                Pose2d tempPose = (RED_ALLIANCE_LEFT || RED_ALLIANCE_RIGHT) ? preloadEnd.plus(temp) : preloadEnd.minus(temp);
                cycle = drive.trajectorySequenceBuilder(preloadEnd)
                        .lineToLinearHeading(tempPose)
                        .lineToLinearHeading(stackPose)
                        .waitSeconds(0.4)
                        .lineToLinearHeading(tempPose)
                        .build();
            }

            for (int i = 0; i < CYCLE_COUNT; i++) {
                drive.followTrajectorySequence(cycle);
                preloadEnd = drive.getPoseEstimate();
            }

            cycleEnd = drive.getPoseEstimate();
        } else {
            cycleEnd = preloadEnd;
        }
    }

    public void park() {
        TrajectorySequence park;
        if (tagOfInterest.id == 2) { //MIDDLE. randomization = 2 or 5
            park = drive.trajectorySequenceBuilder(cycleEnd)
                    .lineToLinearHeading(middleTOI)
                    .build();
        } else if (tagOfInterest.id == 3) { //RIGHT. randomization = 3 or 6
            park = drive.trajectorySequenceBuilder(cycleEnd)
                    .lineToLinearHeading(rightTOI)
                    .build();
        } else { //LEFT. randomization = 1 or 4
            park = drive.trajectorySequenceBuilder(cycleEnd)
                    .lineToLinearHeading(leftTOI)
                    .build();
        }
    }

    public Pose2d reflectOverX(Pose2d pose) {
        double newY = -1 * pose.getY();
        double newHeading = -1 * pose.getHeading();
        pose = new Pose2d(pose.getX(), newY, newHeading);
        return pose;
    }

    public Pose2d reflectOverY(Pose2d pose) {
        double newX = -1 * pose.getX();
        double newHeading = Math.PI - pose.getHeading();
        pose = new Pose2d(newX, pose.getY(), newHeading);
        return pose;
    }

    public Pose2d reflectOverXY(Pose2d pose) {
        reflectOverX(pose);
        reflectOverY(pose);
        return pose;
    }
}