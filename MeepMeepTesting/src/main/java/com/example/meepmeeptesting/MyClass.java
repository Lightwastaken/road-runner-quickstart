package com.example.meepmeeptesting;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicInteger;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MyClass {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48180821614297, 52.48180821614297, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 60.0, Math.toRadians(90)))

                                .strafeLeft(6)
                                .back(48)
                                // preload [ not implmented]
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(-47.5))
                                .forward(6.5)
                                .waitSeconds(0.8)
                                .back(6.5)
                                .waitSeconds(0.3)
                                .turn(Math.toRadians(135))
                                .build()

                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}