package org.firstinspires.ftc.teamcode;

public enum ShootingPosition {
    AGAINST_GOAL(1.05, 0.5056, 0.0),
    CENTER(1.37, 0.2055, 0.0),
    ACROSS_FIELD_TELE(1.87, 0.1222, -0.2),
    ACROSS_FIELD_AUTO(1.87, 0.1222, -0.4655);

    public final double launchMotorSpeed, rackPosition, aimingOffset;

    ShootingPosition(double launchMotorSpeed, double rackPosition, double aimingOffset) {
        this.launchMotorSpeed = launchMotorSpeed;
        this.rackPosition = rackPosition;
        this.aimingOffset = aimingOffset;
    }
}
