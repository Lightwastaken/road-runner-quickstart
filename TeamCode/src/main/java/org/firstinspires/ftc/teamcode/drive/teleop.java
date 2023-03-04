package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotHardware;


@TeleOp(name="GILBYGOBRRR", group="Pushbot")
public class teleop extends LinearOpMode {

    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {

        //checking teleop distance if encoders convert to inches correctly
        final double COUNTS_PER_MOTOR_REV = 5281.1;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 96.0 / 25.4;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        double speedControl = 0.90;
        double initialTime = 0;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {


            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double y = -gamepad1.right_stick_y; // Remember, this is reversed!
            double x = gamepad1.right_stick_x * 1.2; // Counteract imperfect strafing
            double rx = 0;

            if (gamepad1.dpad_right) {
                rx = 1;
            }

            if (gamepad1.dpad_left) {
                rx = -1;
            }
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            if (Math.abs(y) < (x * Math.sqrt(2))) {
                y = 0;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.LF.setPower(frontLeftPower * speedControl);
            robot.LB.setPower(backLeftPower * speedControl);
            robot.RF.setPower(frontRightPower * speedControl);
            robot.RB.setPower(backRightPower * speedControl);

            if (gamepad1.left_trigger > 0.1) {
                speedControl = 1.0;
            }

            if (gamepad1.left_bumper) {
                speedControl = 0.25;
            }

//            if (gamepad1.dpad_up) {
//                robot.claw.setPosition(0.1);
//            }
//
//            if (gamepad1.dpad_down) {
//                robot.claw.setPosition(1);
//            }
//            if (gamepad1.right_bumper) {
//                robot.moveUp();
//            } else if (gamepad1.right_trigger > 0.1) {
//                robot.moveDown();
//            } else {
//                robot.lift(0);
//            }

            if (gamepad1.left_stick_x > 0.5) {
                robot.claw.setPosition(0.5);
            }else if (gamepad1.left_stick_x < -0.5) {
                robot.claw.setPosition(1);
            }

            if (gamepad1.right_trigger > 0.1) {
                robot.lift(0.3);
            } else if (gamepad1.right_bumper) {
                robot.lift(-0.3);
            } else {
                robot.lift(0);
            }

//            if (gamepad1.dpad_down) {
//                robot.initialTime = System.currentTimeMillis();
//
//                robot.RTL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.LTL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                robot.RTL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.LTL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            }
//            }

            telemetry.addData("LF ticks", robot.LF.getCurrentPosition());
            telemetry.addData("LB ticks", robot.LB.getCurrentPosition());
            telemetry.addData("RF ticks", robot.RF.getCurrentPosition());
            telemetry.addData("RB ticks", robot.RB.getCurrentPosition());
            telemetry.addData("LTL ticks", robot.LTL.getCurrentPosition());
            telemetry.addData("RTL ticks", robot.RTL.getCurrentPosition());
            telemetry.addData("LTL power:", robot.LTL.getPower());
            telemetry.addData("RTL power: ", robot.RTL.getPower());
            telemetry.update();
        }

        /*
    public void clawPosition (double position) {
        robot.claw.setPosition(position);
    }
*/
    }
}

