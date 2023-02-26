package org.firstinspires.ftc.teamcode.util;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotHardware;

@Config
@TeleOp
public class LiftPID extends RobotHardware {
    public double integralSum = 0;
    public double lastError = 0;
    public double derivative = 0;
    ElapsedTime timer = new ElapsedTime();

    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0;
    public static double state = RTL.getCurrentPosition();
    public static double reference = 0;

    public void loop() {
        double error = reference - state;
        integralSum += error * timer.seconds();
        derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * p) + (integralSum * i) + (derivative * d) + (Math.cos(Math.toRadians(384.5/360)) * f);

        lift(output);

        telemetry.addData("Lift position", state);
        telemetry.addData("Target", reference);
        telemetry.update();
    }
}
