/*
 * Copyright (c) 2025 Base 10 Assets, LLC
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
 * Neither the name of NAME nor the names of its contributors may be used to
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Main autonomous mode", group = "StarterBot")
public class MainAutonomousMode extends OpMode {

    private static final double TIME_BETWEEN_SHOTS = 2.0;

    private static final double DRIVE_SPEED = 0.5;
    private static final double ROTATE_SPEED = 0.2;
    private static final double MAX_AIMING_TIME = 7.5;
    private static final double RACK_MOVE_TIME = 0.5;

    private ShootingPosition startingPosition = ShootingPosition.AGAINST_GOAL;

    private int shotsToFire = 3; //The number of shots to fire in this auto.

    private double robotRotationAngle = 45;

    private AimingSystem aimingSystem;
    private LaunchSystem launchSystem;
    private DriveSystem driveSystem;
    private AprilTagCam aprilTagCam;
    private RackSystem rackSystem;

    private final ElapsedTime aimingTimeoutTimer = new ElapsedTime();

    private enum AutonomousState {
        AIMING,
        WAIT_FOR_RACK,
        LAUNCHING,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        launchSystem = new LaunchSystem(hardwareMap, "launcher", "left_feeder", "right_feeder");
        launchSystem.setLaunchInterval(TIME_BETWEEN_SHOTS);

        driveSystem = new DriveSystem(hardwareMap, "left_drive", "right_drive");

        rackSystem = new RackSystem(hardwareMap,"rack_control", "rack_button");

        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");
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

        if (gamepad1.a) {
            startingPosition = ShootingPosition.ACROSS_FIELD_AUTO;
        } else if (gamepad1.y) {
            startingPosition = ShootingPosition.AGAINST_GOAL;
        }

        telemetry.addData("\nPress Y", "for next to goal");
        telemetry.addData("Press A", "for across from goal");
        telemetry.addData("Starting position", startingPosition);
    }

    @Override
    public void start() {
        rackSystem.setTargetPosition(startingPosition.rackPosition);
        if (startingPosition == ShootingPosition.ACROSS_FIELD_AUTO) {
            autonomousState = AutonomousState.AIMING;
            aimingTimeoutTimer.reset();
        } else {
            autonomousState = AutonomousState.WAIT_FOR_RACK;
        }
    }

    @Override
    public void loop() {
        boolean rackReady = rackSystem.update(RACK_MOVE_TIME); // update regardless of state to begin moving while robot is aiming
        switch (autonomousState) {
            case AIMING:
                aprilTagCam.update();
                if (aimingSystem.aimForAprilTag(alliance.aprilTagId, aprilTagCam, driveSystem, startingPosition.aimingOffset * alliance.direction) ||
                        aimingTimeoutTimer.seconds() >= MAX_AIMING_TIME) {
                    autonomousState = AutonomousState.WAIT_FOR_RACK;
                }
                break;

            case WAIT_FOR_RACK:
                if (rackReady) {
                    autonomousState = AutonomousState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                launchSystem.update(startingPosition.launchMotorSpeed);
                if (launchSystem.isReady()) {
                    if (shotsToFire > 0) {
                        shotsToFire--;
                        launchSystem.requestLaunch();
                    } else {
                        driveSystem.stopForNewIncrementalTarget();
                        launchSystem.stopMotorIfIdle(); // Can stop it early
                        if (startingPosition == ShootingPosition.AGAINST_GOAL) {
                            autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                        } else {
                            autonomousState = AutonomousState.DRIVING_OFF_LINE;
                        }
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                if (driveSystem.driveIncrementally(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)) {
                    driveSystem.stopForNewIncrementalTarget();
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                if (alliance == Alliance.BLUE) {
                    robotRotationAngle = -45;
                } else if (alliance == Alliance.RED) {
                    robotRotationAngle = 45;
                }

                if (driveSystem.rotateIncrementally(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 1)) {
                    driveSystem.stopForNewIncrementalTarget();
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                double distance = startingPosition == ShootingPosition.AGAINST_GOAL ? -30 : 15;
                if (driveSystem.driveIncrementally(DRIVE_SPEED, distance, DistanceUnit.INCH, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        rackSystem.logStatus(telemetry);
        launchSystem.logStatus(telemetry);
        driveSystem.logStatus(telemetry);
        aimingSystem.logStatus(telemetry);
        telemetry.update();
    }

}