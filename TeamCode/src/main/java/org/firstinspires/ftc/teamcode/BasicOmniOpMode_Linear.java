/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    // Declare OpMode members.

    private PIDController controller;

    private static double p = 0.006, i = 0.001, d = 0.0001;
    private static double f = 0.02;
    public static int target = 0, targetAngle = 0;

    private PIDController controller_drive;
    public static double pd = .013, id = .005, dd = .001;
    public static double driveTarget = 0;

    private double prevTarget = 0;


    private double leftOpen = 0.8, leftClose = 1.0, rightOpen = 0.2, rightClose = 0.0;

    private final double ticks_in_degree = 1425.1/180;

    private DcMotorSimple arm_motor;

    private Servo leftClaw, rightClaw;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftClaw = hardwareMap.get(Servo.class, "gripperLeft");
        rightClaw = hardwareMap.get(Servo.class, "gripperRight");

        controller = new PIDController(p, i ,d);
        controller_drive = new PIDController(pd, id, dd);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorSimple.class, "armMotor");

        boolean clamped = false;
        boolean clampIsPressed = false;
//        arm_motor_mini = hardwareMap.get(RevSPARKMini.class, "armMotor");

        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
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

        // run until the end of the match (driver presses STOP)

        while (opModeIsActive()) {

            /**
             * Arm and claw
             */

            // p = -0.006 i = -0.001 d = -0.0001 f = -0.02
            // ku = 0.03 Tu = 0.139s
            controller.setPID(p, i, d);
            int armPos = leftFrontDrive.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

            double power = pid + ff;

            arm_motor.setPower(-power);

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

            if(gamepad1.right_trigger > .5){
                target = 3300;
            } else if(gamepad1.left_trigger > .5){
                target = 70;
            } else if(gamepad1.a){
                target = 500;
            }

            /**
             * Drive
             */

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double r = Math.atan2(gamepad1.right_stick_x, gamepad1.right_stick_y);

            controller_drive.setPID(pd,id,dd);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;

            if(Math.abs(gamepad1.right_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_y)> 0.1){
                double error = AngleUnit.normalizeDegrees((r*180/Math.PI)-botHeading);
                driveTarget = controller_drive.calculate(error, 0);
            }else{
                driveTarget = 0;
            }
            // 90 0
            // error = normalized error (targetAngle - botheading)
            // calculate(error, 0)


//            driveTarget = controller_drive.calculate(botHeading, targetAngle*Math.PI/180);
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTarget), 1);
            double frontLeftPower = (rotY + rotX - driveTarget) / denominator;
            double backLeftPower = (rotY - rotX - driveTarget) / denominator;
            double frontRightPower = (rotY - rotX + driveTarget) / denominator;
            double backRightPower = (rotY + rotX + driveTarget) / denominator;
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTarget), 1);
//            double frontLeftPower = (rotY + rotX + driveTarget) / denominator;
//            double backLeftPower = (rotY - rotX + driveTarget) / denominator;
//            double frontRightPower = (rotY - rotX - driveTarget) / denominator;
//            double backRightPower = (rotY + rotX - driveTarget) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Angle: ", botHeading);
            telemetry.addData("Right Stick: ", ((r*180)/Math.PI));
            telemetry.addData("PID: ", driveTarget);
            telemetry.addData("Target Angle: ", targetAngle);
            telemetry.addData("", "");

//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            telemetry.addData("Left Claw Pos: ", leftClaw.getPosition());
            telemetry.addData("Right Claw Pos: ", rightClaw.getPosition());
            telemetry.addData("", "");

            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);

            telemetry.update();
        }
    }
}
