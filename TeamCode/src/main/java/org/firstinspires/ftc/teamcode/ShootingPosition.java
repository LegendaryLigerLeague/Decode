package org.firstinspires.ftc.teamcode;

public enum ShootingPosition {
    AGAINST_GOAL(0.93, 0.8389, 0.0),
    ACROSS_FIELD(1.68,0.0666, 0.0),
    CENTER(1.3,0.0666, 0.0),
    ACROSS_FIELD_AUTO(1.68, 0.0666, -2.6);

    public double launchMotorSpeed, rackPosition, aimingOffset;

    ShootingPosition(double launchMotorSpeed, double rackPosition, double aimingOffset) {
        this.launchMotorSpeed = launchMotorSpeed;
        this.rackPosition = rackPosition;
        this.aimingOffset = aimingOffset;
    }
}
