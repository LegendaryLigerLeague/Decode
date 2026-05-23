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

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

@TeleOp(name = "Main teleop mode", group = "StarterBot")
public class MainTeleopMode extends OpMode {

    private final static double SLOWDOWN_MODE_MULTIPLIER = 0.50;
    private static final double MIN_LAUNCH_INTERVAL = 0.5;
    private static final double RACK_MOVE_TIME = 0.5;

    private AprilTagCam aprilTagCam;

    private LaunchSystem launchSystem;
    private DriveSystem driveSystem;
    private RackSystem rackSystem;
    private AimingSystem aimingSystem;

    private Alliance alliance;

    private SaveData saveData;

    private ShootingPosition shootingPosition = ShootingPosition.ACROSS_FIELD_TELE;
    private final Map<ShootingPosition, ShootParams> shootParamsMap = new HashMap<>();

    @Override
    public void init() {
        saveData = new SaveData(hardwareMap);
        alliance = saveData.getAlliance(SaveKey.ALLIANCE, Alliance.RED);

        launchSystem = new LaunchSystem(hardwareMap, "launcher", "left_feeder", "right_feeder");
        launchSystem.setLaunchInterval(MIN_LAUNCH_INTERVAL);

        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");

        driveSystem = new DriveSystem(hardwareMap, "left_drive", "right_drive");
        rackSystem = new RackSystem(hardwareMap, "rack_control", "rack_button");
        aimingSystem = new AimingSystem();

        for (ShootingPosition p : ShootingPosition.values()) {
            ShootParams params = new ShootParams(p.launchMotorSpeed, p.rackPosition, p.aimingOffset);
            shootParamsMap.put(p, params);
        }

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
        telemetry.addData("Selected Alliance", alliance.getTelemetryHtml());
    }

    @Override
    public void loop() {
        aprilTagCam.update();

        adjustSettings();

        ShootParams shootParams = Objects.requireNonNull(shootParamsMap.get(shootingPosition));

        if (gamepad1.left_trigger > 0) {
            aimingSystem.aimForAprilTag(alliance.aprilTagId, aprilTagCam, driveSystem, shootParams.aimingOffset * alliance.direction);
            aimingSystem.logStatus(telemetry);
        } else {
            boolean slowDownMode = gamepad1.right_trigger <= 0;
            driveSystem.driveContinuously(-gamepad1.left_stick_y, -gamepad1.right_stick_x, slowDownMode ? SLOWDOWN_MODE_MULTIPLIER : 1.0);
        }

        rackSystem.setTargetPosition(shootParams.rackPosition);
        if (gamepad1.xWasPressed()) {
            rackSystem.rehome();
        }
        boolean isRackAtTarget = rackSystem.update(RACK_MOVE_TIME);

        if (isRackAtTarget && gamepad1.right_bumper) {
            launchSystem.requestLaunch();
        }
        launchSystem.update(shootParams.launchMotorSpeed);

        logSettings();

        boolean aprilTagDetected = aprilTagCam.isTagDetected(alliance.aprilTagId);
        telemetry.addData("\n" + alliance + " tag", aprilTagDetected ? "Detected" : "Not detected");
        aimingSystem.logStatus(telemetry);

        telemetry.addData("\nCONTROLS",
                "\n\tRB: launch" +
                        "\n\tHold RT: turbo drive" +
                        "\n\tX: rehome the rack" +
                        "\n\tHold LT: aim for alliance's AprilTag" +
                        "\n\tY, B, and A: set shooting position close, center, or far from goal");


        telemetry.addData("\nCHANGING SETTINGS - hold LB +",
                "\n\tD-pad left/right: rack position adjust" +
                        "\n\tD-pad up/down: launch speed adjust" +
                        "\n\tY and A: April tag aiming offset adjust" +
                        "\n\tX and B: Select alliance for auto aiming");

    }

    private void adjustSettings() {
        ShootParams shootParams = Objects.requireNonNull(shootParamsMap.get(shootingPosition));

        if (gamepad1.left_bumper) {
            double rackPositionIncrement = 5 / 180.0; //5 degrees
            if (gamepad1.dpadLeftWasPressed()) {
                shootParams.rackPosition -= rackPositionIncrement;
            } else if (gamepad1.dpadRightWasPressed()) {
                shootParams.rackPosition += rackPositionIncrement;
            }

            if (gamepad1.dpadUpWasPressed()) {
                shootParams.launchMotorSpeed += 0.01;
            } else if (gamepad1.dpadDownWasPressed()) {
                shootParams.launchMotorSpeed -= 0.01;
            }

            if (gamepad1.yWasPressed()) {
                shootParams.aimingOffset += 0.5;
            } else if (gamepad1.aWasPressed()) {
                shootParams.aimingOffset -= 0.5;
            }

            if (gamepad1.bWasPressed()) {
                alliance = Alliance.RED;
            } else if (gamepad1.xWasPressed()) {
                alliance = Alliance.BLUE;
            }
        } else { // LB is not pressed
            if (gamepad1.yWasPressed()) {
                shootingPosition = ShootingPosition.AGAINST_GOAL;
            } else if (gamepad1.aWasPressed()) {
                shootingPosition = ShootingPosition.ACROSS_FIELD_TELE;
            } else if (gamepad1.bWasPressed()) {
                shootingPosition = ShootingPosition.CENTER;
            }
        }
    }

    private void logSettings() {
        telemetry.addData("Selected alliance", alliance.getTelemetryHtml());
        telemetry.addData("Shooting position", shootingPosition.toString() + "\n");

        ShootParams shootParams = Objects.requireNonNull(shootParamsMap.get(shootingPosition));

        telemetry.addData("Launch speed multiplier", shootParams.launchMotorSpeed);
        telemetry.addData("Rack target position", shootParams.rackPosition);
        telemetry.addData("\nApril Tag Offset", shootParams.aimingOffset);
    }

    @Override
    public void stop() {
        saveData.putAlliance(SaveKey.ALLIANCE, alliance);
    }
}