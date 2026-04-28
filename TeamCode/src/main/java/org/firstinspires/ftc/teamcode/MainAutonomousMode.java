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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Main autonomous mode", group = "StarterBot")
public class MainAutonomousMode extends OpMode {

    final static double CLOSE_LAUNCH_SPEED_MULTIPLIER = MainTeleopMode.DEFAULT_CLOSE_LAUNCH_SPEED_MULTIPLIER;
    final static double FAR_LAUNCH_SPEED_MULTIPLIER = MainTeleopMode.DEFAULT_FAR_LAUNCH_SPEED_MULTIPLIER;
    final static double CLOSE_RACK_POSITION = MainTeleopMode.DEFAULT_CLOSE_RACK_POSITION;
    final static double FAR_RACK_POSITION = MainTeleopMode.DEFAULT_FAR_RACK_POSITION;

    private final double TIME_BETWEEN_SHOTS = 2;

    private final double DRIVE_SPEED = 0.5;
    private final double ROTATE_SPEED = 0.2;
    private ShootingPosition startingPosition = ShootingPosition.AGAINST_GOAL;

    private int shotsToFire = 3; //The number of shots to fire in this auto.

    private double robotRotationAngle = 45;
    private double launchSpeedMultiplier;

    private LaunchSystem launchSystem;
    private DriveSystem driveSystem;
    private RackSystem rackSystem;

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    private Alliance alliance = Alliance.RED;

    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;

        launchSystem = new LaunchSystem(hardwareMap, "launcher", "left_feeder", "right_feeder");
        launchSystem.setLaunchInterval(TIME_BETWEEN_SHOTS);

        driveSystem = new DriveSystem(hardwareMap, "left_drive", "right_drive");

        rackSystem = new RackSystem(hardwareMap,"rack_control", "rack_button");

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
            startingPosition = ShootingPosition.ACROSS_FIELD;
            launchSpeedMultiplier = FAR_LAUNCH_SPEED_MULTIPLIER;
            rackSystem.setTargetPosition(FAR_RACK_POSITION);
        } else if (gamepad1.y) {
            startingPosition = ShootingPosition.AGAINST_GOAL;
            launchSpeedMultiplier = CLOSE_LAUNCH_SPEED_MULTIPLIER;
            rackSystem.setTargetPosition(CLOSE_RACK_POSITION);
        }

        telemetry.addData("\nPress Y", "for next to goal");
        telemetry.addData("Press A", "for across from goal");
        telemetry.addData("Starting position", startingPosition);
    }

    @Override
    public void start() {
        // Skip the shooting part if we are across field
        // TODO When far launching works, this can be removed. There will be a new autonomous state
        // that adjusts rack first.
        // TODO we will still need it to skip the drive away from goal state though!
        if (startingPosition == ShootingPosition.ACROSS_FIELD) {
            autonomousState = AutonomousState.DRIVING_OFF_LINE;
        }
    }

    @Override
    public void loop() {
        if (!rackSystem.update()){
            return; // wait until rack in position
        }
        switch (autonomousState) {
            case LAUNCH:
                launchSystem.update(true, launchSpeedMultiplier);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                launchSystem.update(false, launchSpeedMultiplier);
                if (launchSystem.isReady()) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        driveSystem.stopForNewIncrementalTarget();
                        launchSystem.stopMotorIfIdle(); // Can stop it early
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
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
                if (driveSystem.driveIncrementally(DRIVE_SPEED, -30, DistanceUnit.INCH, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        rackSystem.logStatus(telemetry);
        launchSystem.logStatus(telemetry);
        driveSystem.logStatus(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}