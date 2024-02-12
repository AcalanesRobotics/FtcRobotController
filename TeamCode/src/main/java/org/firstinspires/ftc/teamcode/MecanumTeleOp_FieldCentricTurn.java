package org.firstinspires.ftc.teamcode;

import androidx.annotation.MainThread;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class MecanumTeleOp_FieldCentricTurn extends LinearOpMode {
    // Arm PID
    private PIDController armController;

    private double p_arm, i_arm, d_arm;
    private double f_arm;
    public int armTarget;
    private final double ticks_in_degree = 1425.1/180;

    // Turn PID
    private PIDController turnController;
    public static double p_turn = .013, i_turn = .005, d_turn = .001;
    public static double turnTarget = 0;

    // Telemetry
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /**
         * Variables
         */
        // Drive Motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        DcMotorSimple arm_motor = hardwareMap.get(DcMotorSimple.class, "armMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Claw servos
        Servo leftClaw = hardwareMap.get(Servo.class, "gripperLeft");
        Servo rightClaw = hardwareMap.get(Servo.class, "gripperRight");

        // Arm PID
        p_arm = 0.006;
        i_arm = 0.001;
        d_arm = 0.0001;
        f_arm = 0.02;
        armTarget = 0;

        // Turn PID

        // Claw values
        boolean clamped = false;
        boolean clampIsPressed = false;

        double leftOpen = 0.8, leftClose = 1.0, rightOpen = 0.2, rightClose = 0.0;

        /**
         * PID Init
         */
        armController = new PIDController(p_arm, i_arm, d_arm);
        turnController = new PIDController(p_turn, i_turn, d_turn);

        /**
         * IMU Init
         */
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            /**
             * Gamepad Variables
             */
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double r = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y);

            /**
             * Control
             */
            //IMU Reset
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            // Arm control
            if(gamepad1.right_trigger > .5){
                armTarget = -3300;
            } else if(gamepad1.left_trigger > .5){
                armTarget = 0 ;
            } else if(gamepad1.a){
                armTarget = -500;
            }

            // Arm PID
            int armPos = frontLeftMotor.getCurrentPosition();
            double pid = armController.calculate(armPos, armTarget);
            double ff = Math.cos(Math.toRadians(armTarget/ticks_in_degree)) * f_arm;

            double power = pid + ff;

            arm_motor.setPower(power);

            // Claw
            if(gamepad1.right_bumper && !clamped && !clampIsPressed){
                clamped = true;
                clampIsPressed = true;
                leftClaw.setPosition(leftClose);
                rightClaw.setPosition(rightClose);
            }
            else if(gamepad1.right_bumper && clamped && !clampIsPressed){
                clamped = false;
                clampIsPressed = true;
                leftClaw.setPosition(leftOpen);
                rightClaw.setPosition(rightOpen);
            }else if(!gamepad1.right_bumper && clampIsPressed){
                clampIsPressed = false;
            }

            /**
             * Drive Control
             */
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Turn PID
            turnController.setPID(p_turn, i_turn, d_turn);

            double error = AngleUnit.normalizeDegrees((turnTarget*Math.PI/180)-botHeading);
            double turnOut = turnController.calculate(error, 0);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnOut), 1);
            double frontLeftPower = (rotY + rotX - turnOut) / denominator;
            double backLeftPower = (rotY - rotX - turnOut) / denominator;
            double frontRightPower = (rotY - rotX + turnOut) / denominator;
            double backRightPower = (rotY + rotX + turnOut) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("pos", armPos);
            telemetry.addData("target", armTarget);

            telemetry.addData("", "");

            telemetry.addData("right stick: ", r*180/Math.PI);
            telemetry.addData("turn target: ", turnTarget);
            telemetry.addData("imu yaw: ", botHeading*180/Math.PI);

            telemetry.update();
        }
    }
}