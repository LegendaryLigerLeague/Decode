package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchSystem {
    //The feeder servos run this long when a shot is requested.
    private final static double FEED_TIME_SECONDS = 0.35;

    //After feeding a ball, the servos reverse to ensure there isn't a preloaded ball.
    private final static double REVERSE_FEED_SECONDS = 0.20;
    private final static double STOP_SPEED = 0.0;
    private final static double FULL_SPEED = 1.0;

    private final static double LAUNCHER_MOTOR_TIMEOUT = 3;
    private final static double LAUNCHER_TARGET_VELOCITY = 1125;
    private final static double LAUNCHER_MIN_VELOCITY = 1075;

    private final DcMotorEx launcher;
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;

    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime launcherTimer = new ElapsedTime();

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        REVERSE_FEED
    }

    private LaunchState launchState = LaunchState.IDLE;

    private double launchInterval = 0.0;

    public LaunchSystem(HardwareMap hardwareMap, String launcherName, String leftFeederName, String rightFeederName) {
        launcher = hardwareMap.get(DcMotorEx.class, launcherName);
        leftFeeder = hardwareMap.get(CRServo.class, leftFeederName);
        rightFeeder = hardwareMap.get(CRServo.class, rightFeederName);

        launcher.setZeroPowerBehavior(BRAKE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

    }

    public void update(boolean requestLaunch, double launchSpeedMultiplier) {
        // Stop the launcher motor when we haven't launched in a while so it isn't turning while loading
        if (launcherTimer.seconds() >= LAUNCHER_MOTOR_TIMEOUT) {
            launcherTimer.reset(); // so we don't keep stopping it while trying to start next launch`
            launcher.setVelocity(STOP_SPEED);
        }

        switch (launchState) {
            case IDLE:
                if (requestLaunch) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY * launchSpeedMultiplier);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY * launchSpeedMultiplier) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launcherTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.REVERSE_FEED;
                    feederTimer.reset();
                }
                break;
            case REVERSE_FEED:
                leftFeeder.setPower(-FULL_SPEED);
                rightFeeder.setPower(-FULL_SPEED);
                if (feederTimer.seconds() > REVERSE_FEED_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                }
                if (feederTimer.seconds() > Math.max(REVERSE_FEED_SECONDS, launchInterval)) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public double getLaunchInterval() {
        return launchInterval;
    }

    public void setLaunchInterval(double launchInterval) {
        this.launchInterval = launchInterval;
    }

    public void stopMotorIfIdle() {
        if (launchState == LaunchState.IDLE) {
            launcher.setVelocity(STOP_SPEED);
        }
    }

    public boolean isReady() {
        return launchState == LaunchState.IDLE;
    }

    public LaunchState getState() {
        return launchState;
    }

    public double getLaunchMotorVelocity() {
        return launcher.getVelocity();
    }
}
