/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */
@Config
public class RobotHardware {
    public SampleMecanumDrive drive;
    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx LF; //left front(chassis)
    public DcMotorEx LB; //left back(chassis)
    public DcMotorEx RF; //right front(chassis)
    public DcMotorEx RB; //right back(chassis)
    public static DcMotorEx RTL; //right motor(lift)
    public DcMotorEx LTL; //left motor(lift)
    public Servo claw; //claw
    public DistanceSensor sensor;
    public ServoImplEx v4bLeft;
    public ServoImplEx v4bRight;


    public ElapsedTime runtime = new ElapsedTime();
    public Orientation lastAngles = new Orientation();
    public double currAngle = 0.0;
//    public static  double GROUND_OUTTAKE_POSITION = 200;
//    public static  double BOTTOM_OUTTAKE_POSITION = 0;
//    public static  double LOW_OUTTAKE_POSITION = 550;
//    public static  double MID_OUTTAKE_POSITION = 900;
//    public static  double TOP_OUTTAKE_POSITION = 1230;
    public static final int BOTTOM = 0;
    public static final int GROUND_JUNC = 49;
    public static final int LOW_JUNC = 1100;
    public static final int MID_JUNC = 1955;
    public static final int HIGH_JUNC = 2770;
    public static final double OUTTAKE_SPEED = 30 * 5281.1 / 60; //RPM * ENCODER TICKS PER REV / 60
    static final double COUNTS_PER_MOTOR_REV = 5281.1;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 96/25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.7;
    public static double  targetPosition = 550;
    private double initalTime = System.currentTimeMillis();

    public static  double strafeV = 4;
    public static  double foward1 = 41.5;
    public static  double vectorx1= -22;
    public static  double vectory1= 15.5;
    //    Thread liftPID = new Thread(new RobotHardware());
    public double initialTime = System.currentTimeMillis();

    public double integralSum = 0;
    public double lastError = 0;
    public double derivative = 0;
    ElapsedTime timer = new ElapsedTime();

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public RobotHardware() {}


    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        //INITIALIZE ALL HARDWARE
        LF  = myOpMode.hardwareMap.get(DcMotorEx.class, "LF");
        LB = myOpMode.hardwareMap.get(DcMotorEx.class, "LB");
        RF  = myOpMode.hardwareMap.get(DcMotorEx.class, "RF");
        RB = myOpMode.hardwareMap.get(DcMotorEx.class, "RB");
        RTL = myOpMode.hardwareMap.get(DcMotorEx.class, "RTL");
        LTL = myOpMode.hardwareMap.get(DcMotorEx.class, "LTL");
        claw = myOpMode.hardwareMap.get(Servo.class, "CLAW");
        v4bLeft = myOpMode.hardwareMap.get(ServoImplEx.class, "V4B LEFT");
        v4bRight = myOpMode.hardwareMap.get(ServoImplEx.class, "V4B RIGHT");
//        sensor = myOpMode.hardwareMap.get(DistanceSensor.class, "distance sensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        RTL.setDirection(DcMotorEx.Direction.FORWARD);
        LTL.setDirection(DcMotorEx.Direction.REVERSE);

        //ALL MOTORS RUN WITH ENCODERS
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setPosition(1);
        setMotorPowers(0);

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
    }

    public void setMotorPowers(double speed) {
        // Output the values to the motor drives.
        LF.setPower(speed);
        LB.setPower(speed);
        RF.setPower(speed);
        RB.setPower(speed);
    }

    public boolean isChassisVeloZero() {
        return LF.getVelocity() == 0 && LB.getVelocity() == 0 && RF.getVelocity() == 0 && RB.getVelocity() == 0;
    }

    public void lift(double power){
        RTL.setPower(power);
        LTL.setPower(power);
    }

    public void reset() {
        RTL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LTL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * p) + (integralSum * i) + (derivative * d);

        RTL.setVelocity(output);
        LTL.setVelocity(output);
    }

    public void liftTo(int pos) {
        RTL.setTargetPosition(pos);
        LTL.setTargetPosition(pos);

        RTL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LTL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (RTL.getCurrentPosition() < pos) {
            PIDControl(pos, RTL.getCurrentPosition());
        }

        RTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void virtualSetPos(double pos) {
        v4bRight.setPosition(pos);
        v4bLeft.setPosition(pos);
    }

    public boolean modeNameContains(String str) {
        return getClass().getName().contains(str);
    }
}

