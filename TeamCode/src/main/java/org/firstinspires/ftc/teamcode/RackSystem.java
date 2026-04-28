package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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

    public RackSystem(HardwareMap hardwareMap, String servoName, String touchSensorName){
        servo = hardwareMap.get(Servo.class, servoName);
        touchSensor = hardwareMap.get(TouchSensor.class, touchSensorName);
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void rehome(){
        state = State.REHOMING;
    }

    /**
     * Called in loop
     * @return true if rack is at target position.
     */
    public boolean update() {
        switch (state) {
            case REHOMING:
                servo.setPosition(servo.getPosition() + 1.0/360.0); // Set to high value but move slowly.
                if (touchSensor.isPressed()) {
                    homePosition = servo.getPosition();
                    state = State.SEEKING_TARGET;
                }
                break;
            case SEEKING_TARGET:
                double targetServoPos = homePosition - targetPosition;
                servo.setPosition(targetServoPos);
                if (servo.getPosition() == targetServoPos) {
                    return true;
                }
                break;
        }
        return false;
    }

    public void logStatus(Telemetry telemetry) {
        telemetry.addData("Rack servo state", state);
    }
}
