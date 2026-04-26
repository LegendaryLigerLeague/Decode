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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name = "Main teleop mode", group = "StarterBot")
public class MainTeleopMode extends OpMode {

    final static double DEFAULT_CLOSE_LAUNCH_SPEED_MULTIPLIER = .93;
    final static double DEFAULT_FAR_LAUNCH_SPEED_MULTIPLIER = 1.82;

    final static double DEFAULT_CLOSE_RACK_POSITION = 0.95;
    final static double DEFAULT_FAR_RACK_POSITION = 0.4;
    final double SLOWDOWN_MODE_MULTIPLIER = 0.50;

    private AprilTagCam aprilTagCam;

    private LaunchSystem launchSystem;
    private DriveSystem driveSystem;
    private Alliance alliance = Alliance.RED;
    private RackSystem rackSystem;

    private double closeTargetRackPosition = DEFAULT_CLOSE_RACK_POSITION;
    private double farTargetRackPosition = DEFAULT_FAR_RACK_POSITION;
    double closeLaunchSpeedMultiplier = DEFAULT_CLOSE_LAUNCH_SPEED_MULTIPLIER;
    double farLaunchSpeedMultiplier = DEFAULT_FAR_LAUNCH_SPEED_MULTIPLIER;
    private ShootingPosition shootingPosition = ShootingPosition.ACROSS_FIELD;

    @Override
    public void init() {


        launchSystem = new LaunchSystem(hardwareMap, "launcher", "left_feeder", "right_feeder");
        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");

        driveSystem = new DriveSystem(hardwareMap, "left_drive", "right_drive");
        rackSystem = new RackSystem(hardwareMap,"rack_control", "rack_button");


        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper) {
            if (gamepad1.leftBumperWasPressed()) {
                driveSystem.stopForNewIncrementalTarget();
            }
            aimForAprilTag();
        } else {
            boolean slowDownMode = gamepad1.right_trigger <= 0;
            driveSystem.driveContinuously(-gamepad1.left_stick_y, -gamepad1.right_stick_x, slowDownMode ? SLOWDOWN_MODE_MULTIPLIER : 1.0);
        }

        if (gamepad1.yWasPressed()) {
            shootingPosition = ShootingPosition.AGAINST_GOAL;
        } else if (gamepad1.aWasPressed()) {
            shootingPosition = ShootingPosition.ACROSS_FIELD;
        }

        if (gamepad1.dpadUpWasPressed() && gamepad1.left_bumper) {
            switch (shootingPosition) {
                case ACROSS_FIELD:
                    farLaunchSpeedMultiplier += 0.01;
                    break;
                case AGAINST_GOAL:
                    closeLaunchSpeedMultiplier += 0.1;
                    break;
            }
        } else if (gamepad1.dpadDownWasPressed() && gamepad1.left_bumper) {
            switch (shootingPosition) {
                case ACROSS_FIELD:
                    farLaunchSpeedMultiplier -= 0.01;
                    break;
                case AGAINST_GOAL:
                    closeLaunchSpeedMultiplier -= 0.1;
                    break;
            }
        }

        rackSystem.setTargetPosition(
                shootingPosition == ShootingPosition.AGAINST_GOAL ? closeTargetRackPosition : farTargetRackPosition
        );
        if (gamepad1.xWasPressed()) {
            rackSystem.rehome();
        }
        boolean isRackAtTarget = rackSystem.update();

        double launchSpeedMultiplier =
                shootingPosition == ShootingPosition.AGAINST_GOAL ? closeLaunchSpeedMultiplier : farLaunchSpeedMultiplier;
        boolean shouldRequestLaunch = isRackAtTarget && gamepad1.rightBumperWasPressed();
        launchSystem.update(shouldRequestLaunch, launchSpeedMultiplier);

        double rackPositionIncrement = 10 / 180.0; //10 degrees
        if (gamepad1.dpadLeftWasPressed() && gamepad1.left_bumper) {
            switch (shootingPosition) {
                case ACROSS_FIELD:
                    farTargetRackPosition -= rackPositionIncrement;
                    break;
                case AGAINST_GOAL:
                    closeTargetRackPosition -= rackPositionIncrement;
                    break;
            }
        } else if (gamepad1.dpadRightWasPressed() && gamepad1.left_bumper) {
            switch (shootingPosition) {
                case ACROSS_FIELD:
                    farTargetRackPosition += rackPositionIncrement;
                    break;
                case AGAINST_GOAL:
                    closeTargetRackPosition += rackPositionIncrement;
                    break;
            }
        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Shooting position", shootingPosition);
        telemetry.addData("\nLaunch state", launchSystem.getState());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", driveSystem.getLeftMotorPower(), driveSystem.getRightMotorPower());
        telemetry.addData("motorSpeed", launchSystem.getLaunchMotorVelocity());
        switch (shootingPosition) {
            case ACROSS_FIELD:
                telemetry.addData("Launch speed multiplier", farLaunchSpeedMultiplier);
                telemetry.addData("Rack target position", farTargetRackPosition);
                break;
            case AGAINST_GOAL:
                telemetry.addData("Launch speed multiplier", farLaunchSpeedMultiplier);
                telemetry.addData("Rack target position", closeTargetRackPosition);
                break;
        }
        telemetry.addData("Rack home position", rackSystem.getHomePosition());
        telemetry.addData("Rack servo actual position", rackSystem.getServoPosition());
        telemetry.addData("Rack servo set target position", rackSystem.getTargetPosition());
        telemetry.addData("Rack servo state", rackSystem.getState());
        telemetry.addData("Rack at target", isRackAtTarget);

        boolean aprilTagDetected = aprilTagCam.isTagDetected(alliance.aprilTagId);
        telemetry.addData("\n\n" + alliance + " tag", aprilTagDetected ? "Detected" : "Not detected");
        if (aprilTagDetected) {
            telemetry.addData("Tag bearing", aprilTagCam.getBearingToTag(alliance.aprilTagId));
            telemetry.addData("Tag distance", aprilTagCam.getDistanceToTag(alliance.aprilTagId));
        }

        telemetry.addData("\n\nCONTROLS:",
                "\nRight bumper: launch" +
                        "\nHold right trigger: turbo" +
                        "\nHold LB + D-pad left and right: manual rack position adjust" +
                        "\nHold LB + D-pad up or down: launch speed multiplier adjust" +
                        "\nX: request rehome of rack" +
                        "\nHold left trigger: aim for alliance's AprilTag" +
                        "\nY and A: set shooting position close or far from goal"
        );

    }

    @Override
    public void stop() {
    }

    private void aimForAprilTag() {


    }


}