package org.firstinspires.ftc.teamcode;

public enum Alliance {
    RED(1.0, AprilTagId.RED), BLUE(-1.0, AprilTagId.BLUE);

    /** The bearing direction to turn toward the goal if facing the Obelisk. */
    final double direction;
    final AprilTagId aprilTagId;
    Alliance(double direction, AprilTagId aprilTagId) {
        this.direction = direction;
        this.aprilTagId = aprilTagId;
    }

    public Alliance getOpposite() {
        if (this == RED) return BLUE;
        return RED;
    }

    public String getTelemetryHtml() {
        return "<font color=\"" + name().toLowerCase() + "\">" + name() + "</font>";
    }

}
