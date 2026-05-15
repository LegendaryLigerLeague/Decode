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

    public RackSystem(HardwareMap hardwareMap, String servoName, String touchSensorName){
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

    public void rehome(){
        state = State.REHOMING;
    }

    /**
     * Called in loop
     * @return true if holdTime seconds have passed since the target was last changed after homing
     * completed. We use a hold time instead of checking servo position because the servo cannot
     * report if it has reached its set position.
     */
    public boolean update(double holdTime) {
        switch (state) {
            case REHOMING:
                servo.setPosition(servo.getPosition() + 1/360.0); // Set to high value but move slowly to avoid slamming button.
                if (touchSensor.isPressed()) {
                    homePosition = servo.getPosition();
                    state = State.SEEKING_TARGET;
                    timer.reset();
                }
                break;
            case SEEKING_TARGET:
                double targetServoPos = homePosition - targetPosition;
                servo.setPosition(targetServoPos);
                if (timer.seconds() >= holdTime) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void setServoToRackLoadingPosition(){
        servo.setPosition(0.0);
    }

    public void logStatus(Telemetry telemetry) {
        telemetry.addData("Rack servo state", state);
//        telemetry.addData("Rack servo position", servo.getPosition());
//        telemetry.addData("Rack servo's determined home position", homePosition);
//        telemetry.addData("Rack touch sensor status", touchSensor.isPressed());
    }
}
