package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RackSystem {
    public enum State {
        REHOMING,
        SEEKING_TARGET;
    }

    private State state = State.REHOMING;
    private double targetPosition = 0.0;
    private double homePosition = 0.5;
    private final Servo servo;
    private final TouchSensor touchSensor;
    private final ElapsedTime timer = new ElapsedTime();

    public RackSystem(HardwareMap hardwareMap, String servoName, String touchSensorName) {
        servo = hardwareMap.get(Servo.class, servoName);
        touchSensor = hardwareMap.get(TouchSensor.class, touchSensorName);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        if (this.targetPosition == targetPosition) {
            return;
        }
        timer.reset();
        this.targetPosition = targetPosition;
    }

    public void rehome() {
        state = State.REHOMING;
    }

    /**
     * Called in loop
     *
     * @return true if holdTime seconds have passed since the target was last changed after homing
     * completed. We use a hold time instead of checking servo position because the servo cannot
     * report if it has reached its set position.
     */
    public boolean update(double holdTime) {
        switch (state) {
            case REHOMING:
                setServoPositionClamped(servo.getPosition() + 1 / 360.0); // Set to high value but move slowly to avoid slamming button.
                if (touchSensor.isPressed()) {
                    homePosition = servo.getPosition();
                    state = State.SEEKING_TARGET;
                    timer.reset();
                }
                break;
            case SEEKING_TARGET:
                double targetServoPos = homePosition - targetPosition;
                setServoPositionClamped(targetServoPos);
                if (timer.seconds() >= holdTime) {
                    return true;
                }
                break;
        }
        return false;
    }

    private void setServoPositionClamped(double position) {
        servo.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    /**
     * Cycles the rack to both extremes and stops at 0.2, where the rack can be aligned to the outer
     * edge of robot frame before engaging the gears.
     */
    public boolean cycleToLoadingPosition() {
        double interval = 0.75;
        if (timer.seconds() <= interval) {
            servo.setPosition(0.0);
        } else if (timer.seconds() <= interval * 2) {
            servo.setPosition(1.0);
        } else if (timer.seconds() <= interval * 3){
            servo.setPosition(0.2);
        } else {
            return true;
        }
        return false;
    }

    public void logStatus(Telemetry telemetry) {
        telemetry.addData("Rack servo state", state);
        telemetry.addData("Rack servo position", servo.getPosition());
        telemetry.addData("Rack servo's determined home position", homePosition);
        telemetry.addData("Rack touch sensor status", touchSensor.isPressed());
    }
}
