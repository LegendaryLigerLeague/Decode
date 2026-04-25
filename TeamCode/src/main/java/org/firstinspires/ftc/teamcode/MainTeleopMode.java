/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Main teleop mode", group = "StarterBot")
//@Disabled
public class MainTeleopMode extends OpMode {

    final static double DEFAULT_LAUNCH_SPEED_MULTIPLIER = .93;

    final double SLOWDOWN_MODE_MULTIPLIER = 0.50;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private Servo rackControlServo = null;
    private TouchSensor rackTouchSensor;

    enum RackServoState {
        IDLE,
        FINDING_BUTTON,
        GOING_TO_TARGET;
    }

    private RackServoState rackServoState;
    private Servo rackControl = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    double launchSpeedMultiplier = DEFAULT_LAUNCH_SPEED_MULTIPLIER;
    double targetRackPosition = 0.0;
    double rackHomePosition = 0.5;

    private AprilTagCam aprilTagCam;

    private LaunchSystem launchSystem;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        rackServoState = RackServoState.IDLE;
        launchSystem = new LaunchSystem(hardwareMap, "launcher", "left_feeder", "right_feeder");
        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");
        
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        rackControlServo = hardwareMap.get(Servo.class, "rack_control");
        rackTouchSensor = hardwareMap.get(TouchSensor.class, "rack_button");
        rackControl = hardwareMap.get(Servo.class, "rack_control");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        arcadeDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_x, gamepad1.right_trigger <= 0);

        if (gamepad1.dpadUpWasPressed() && gamepad1.left_bumper) {
            launchSpeedMultiplier += 0.01;
        } else if (gamepad1.dpadDownWasPressed() && gamepad1.left_bumper) {
            launchSpeedMultiplier -= 0.01;
        }

        launchSystem.update(gamepad1.rightBumperWasPressed(), launchSpeedMultiplier);

        double rackPositionIncrement = 10 / 180.0; //10 degrees
        if (gamepad1.dpadLeftWasPressed()) {
            targetRackPosition -= rackPositionIncrement;
        } else if (gamepad1.dpadRightWasPressed()) {
            targetRackPosition += rackPositionIncrement;
        } else {
            updateRackServo(gamepad1.xWasPressed());
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Launch state", launchSystem.getState());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("motorSpeed", launchSystem.getLaunchMotorVelocity());
        telemetry.addData("Launch speed multiplier", launchSpeedMultiplier);
        telemetry.addData("Rack target Position", targetRackPosition);
        telemetry.addData("Rack home position", rackHomePosition);
        telemetry.addData("Rack servo actual position", rackControlServo.getPosition());
        telemetry.addData("Rack servo state", rackServoState);

        telemetry.addData("\n\nCONTROLS:",
                "\nRight bumper: launch" +
                        "\nHold right trigger: turbo" +
                        "\nD-pad left and right: manual rack position adjust" +
                        "\nHold LB + D-pad up or down: launch speed multiplier adjust" +
                        "\nX: request rehome of rack"
        );

    }

    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double rotate, boolean isSlowdownMode) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;
        if (isSlowdownMode) {
            leftPower = leftPower * SLOWDOWN_MODE_MULTIPLIER;
            rightPower = rightPower * SLOWDOWN_MODE_MULTIPLIER;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    private boolean updateRackServo(boolean rehomeRequested) {
        switch (rackServoState) {
            case IDLE:
                if (rehomeRequested) {
                    rackServoState = RackServoState.FINDING_BUTTON;
                } else if (rackControlServo.getPosition() != rackHomePosition - targetRackPosition) {
                    rackServoState = RackServoState.GOING_TO_TARGET;
                }
                break;
            case FINDING_BUTTON:
                rackControlServo.setPosition(1.0);
                if (rackTouchSensor.isPressed()) {
                    rackHomePosition = rackControlServo.getPosition();
                    rackServoState = RackServoState.GOING_TO_TARGET;
                }
                break;
            case GOING_TO_TARGET:
                if (rehomeRequested) {
                    rackServoState = RackServoState.IDLE;
                    break;
                }
                double targetServoPos = rackHomePosition - targetRackPosition;
                rackControlServo.setPosition(targetServoPos);
                if (rackControlServo.getPosition() == targetServoPos) {
                    rackServoState = RackServoState.IDLE;
                    return true;
                }
                break;
        }
        return false;
    }
}