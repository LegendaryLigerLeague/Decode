package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AimingSystem {

    /** Aiming is considered complete when bearing is below this many degrees positive or negative. */
    private static final double ANGLE_THRESHOLD = 0.5;
    private final double ROTATE_SPEED = 0.3;
    private final double HOLD_SECONDS = 0.2;

    private boolean rotating = false;
    private double bearing = 0.0;

    /**
     * Attempts to rotate so the given AprilTag is centered to the camera. Takes over the drive system,
     * so do not call this while sending other drive commands to the drive system.
     * @return True if the tag is centered or is undetected while stationary.
     */
    public boolean aimForAprilTag(AprilTagId id, AprilTagCam cam, DriveSystem driveSystem) {
        if (!rotating) {
            bearing = -cam.getBearingToTag(id); // We flip the bearing because the april tag cam uses opposite coordinate system
            if (bearing > ANGLE_THRESHOLD || bearing < ANGLE_THRESHOLD) {
                driveSystem.stopForNewIncrementalTarget();
                rotating = true;
            } else {
                return true;
            }
        }

        if (driveSystem.rotateIncrementally(ROTATE_SPEED, bearing, AngleUnit.DEGREES, HOLD_SECONDS)) {
            rotating = false;
        }
        return false;
    }

    public void logStatus(Telemetry telemetry) {
        telemetry.addData("Aiming bearing (non-zero when tag detected)", bearing);
        telemetry.addData("Aiming system is rotating", rotating);
    }
}
