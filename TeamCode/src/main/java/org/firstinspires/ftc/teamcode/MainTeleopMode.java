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

@TeleOp(name = "Main teleop mode", group = "StarterBot")
public class MainTeleopMode extends OpMode {

    final static double DEFAULT_CLOSE_LAUNCH_SPEED_MULTIPLIER = .93;
    final static double DEFAULT_FAR_LAUNCH_SPEED_MULTIPLIER = 1.82;

    final static double DEFAULT_CLOSE_RACK_POSITION = 0.8389;
    final static double DEFAULT_FAR_RACK_POSITION = 0.3444;
    final double SLOWDOWN_MODE_MULTIPLIER = 0.50;

    private static final double MIN_LAUNCH_INTERVAL = 0.5;
    private static final double RACK_MOVE_TIME = 0.5;

    private AprilTagCam aprilTagCam;

    private LaunchSystem launchSystem;
    private DriveSystem driveSystem;
    private RackSystem rackSystem;
    private AimingSystem aimingSystem;

    private Alliance alliance = Alliance.RED;

    private SaveData saveData;

    private double closeTargetRackPosition;
    private double farTargetRackPosition;
    double closeLaunchSpeedMultiplier;
    double farLaunchSpeedMultiplier;
    private ShootingPosition shootingPosition = ShootingPosition.ACROSS_FIELD;

    @Override
    public void init() {
        saveData = new SaveData(telemetry);
        closeTargetRackPosition = saveData.getDouble(SaveKey.CLOSE_RACK_POSITION, DEFAULT_CLOSE_RACK_POSITION);
        farTargetRackPosition = saveData.getDouble(SaveKey.FAR_RACK_POSITION, DEFAULT_FAR_RACK_POSITION);
        closeLaunchSpeedMultiplier = saveData.getDouble(SaveKey.CLOSE_LAUNCH_MULTIPLIER, DEFAULT_CLOSE_LAUNCH_SPEED_MULTIPLIER);
        farLaunchSpeedMultiplier = saveData.getDouble(SaveKey.FAR_LAUNCH_MULTIPLIER, DEFAULT_FAR_LAUNCH_SPEED_MULTIPLIER);
        alliance = saveData.getAlliance(SaveKey.ALLIANCE, Alliance.RED);

        launchSystem = new LaunchSystem(hardwareMap, "launcher", "left_feeder", "right_feeder");
        launchSystem.setLaunchInterval(MIN_LAUNCH_INTERVAL);

        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");

        driveSystem = new DriveSystem(hardwareMap, "left_drive", "right_drive");
        rackSystem = new RackSystem(hardwareMap,"rack_control", "rack_button");
        aimingSystem = new AimingSystem();

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
    public void loop() {
        aprilTagCam.update();

        if (gamepad1.left_trigger > 0) {
            aimingSystem.aimForAprilTag(alliance.aprilTagId, aprilTagCam, driveSystem);
            aimingSystem.logStatus(telemetry);
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
                    closeLaunchSpeedMultiplier += 0.01;
                    break;
            }
        } else if (gamepad1.dpadDownWasPressed() && gamepad1.left_bumper) {
            switch (shootingPosition) {
                case ACROSS_FIELD:
                    farLaunchSpeedMultiplier -= 0.01;
                    break;
                case AGAINST_GOAL:
                    closeLaunchSpeedMultiplier -= 0.01;
                    break;
            }
        }

        rackSystem.setTargetPosition(
                shootingPosition == ShootingPosition.AGAINST_GOAL ? closeTargetRackPosition : farTargetRackPosition
        );
        if (gamepad1.xWasPressed()) {
            rackSystem.rehome();
        }
        boolean isRackAtTarget = rackSystem.update(RACK_MOVE_TIME);

        double launchSpeedMultiplier =
                shootingPosition == ShootingPosition.AGAINST_GOAL ? closeLaunchSpeedMultiplier : farLaunchSpeedMultiplier;
        if (isRackAtTarget && gamepad1.right_bumper) {
            launchSystem.requestLaunch();
        }
        launchSystem.update(launchSpeedMultiplier);

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

        telemetry.addData("Shooting position", shootingPosition.toString() + "\n");

        switch (shootingPosition) {
            case ACROSS_FIELD:
                telemetry.addData("Launch speed multiplier", farLaunchSpeedMultiplier);
                telemetry.addData("Rack target position", farTargetRackPosition);
                break;
            case AGAINST_GOAL:
                telemetry.addData("Launch speed multiplier", closeLaunchSpeedMultiplier);
                telemetry.addData("Rack target position", closeTargetRackPosition);
                break;
        }

        telemetry.addData("\nRack at target", isRackAtTarget);

        boolean aprilTagDetected = aprilTagCam.isTagDetected(alliance.aprilTagId);
        telemetry.addData("\n" + alliance + " tag", aprilTagDetected ? "Detected" : "Not detected");
        if (aprilTagDetected) {
            aimingSystem.logStatus(telemetry);
        }

        telemetry.addData("\nCONTROLS",
                "\n\tRight bumper: launch" +
                        "\n\tHold RT: turbo drive" +
                        "\n\tHold LB + D-pad left/right: rack position adjust" +
                        "\n\tHold LB + D-pad up/down: launch speed adjust" +
                        "\n\tX: rehome the rack" +
                        "\n\tHold LT: aim for alliance's AprilTag" +
                        "\n\tY and A: set shooting position close or far from goal"
        );

    }

    @Override
    public void stop() {
        saveData.putDouble(SaveKey.FAR_RACK_POSITION, farTargetRackPosition);
        saveData.putDouble(SaveKey.CLOSE_RACK_POSITION, closeTargetRackPosition);
        saveData.putDouble(SaveKey.FAR_LAUNCH_MULTIPLIER, farLaunchSpeedMultiplier);
        saveData.putDouble(SaveKey.CLOSE_LAUNCH_MULTIPLIER, closeLaunchSpeedMultiplier);
        saveData.putAlliance(SaveKey.ALLIANCE, alliance);
        saveData.save();
    }

}