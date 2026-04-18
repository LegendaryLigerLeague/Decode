package org.firstinspires.ftc.teamcode;

public enum TeamSide {
    RED(1.0, AprilTagId.RED), BLUE(-1.0, AprilTagId.BLUE);

    final double direction;
    final AprilTagId aprilTagId;
    TeamSide(double direction, AprilTagId aprilTagId) {
        this.direction = direction;
        this.aprilTagId = aprilTagId;
    }

    public TeamSide getOpposite() {
        if (this == RED) return BLUE;
        return RED;
    }
}
