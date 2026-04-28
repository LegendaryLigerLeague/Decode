package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagCam {

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private List<AprilTagDetection> detections = new ArrayList<>();
    private final Telemetry telemetry;

    public AprilTagCam(HardwareMap hardwareMap, Telemetry telemetry, String cameraName) {
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .build();

    }

    @SuppressLint("DefaultLocale")
    public void printDataForTag(AprilTagDetection detection) {
        if (detection == null) return;
        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }

    /** Must call this in the loop. */
    public void update() {
        detections = aprilTagProcessor.getDetections();
    }

    public AprilTagDetection getDetectedTagOrNull(AprilTagId id) {
        if (id == null) return null;
        for (AprilTagDetection detection: detections) {
            if (detection.id == id.id) {
                return detection;
            }
        }
        return null;
    }

    public boolean isTagDetected(AprilTagId id) {
        if (id == null) return false;
        return getDetectedTagOrNull(id) != null;
    }

    /**
     *
     * @return Bearing in degrees from center of camera. Positive number is right.
     * of center. 0.0 if the tag isn't detected.
     */
    public double getBearingToTag(AprilTagId id) {
        if (id == null) return 0.0;
        AprilTagDetection detection = getDetectedTagOrNull(id);
        if (detection == null) return 0.0;
        return detection.ftcPose.bearing;
    }

    /**
     *
     * @return Distance from tag in inches. 0.0 if the tag isn't detected.
     */
    public double getDistanceToTag(AprilTagId id) {
        if (id == null) return 0.0;
        AprilTagDetection detection = getDetectedTagOrNull(id);
        if (detection == null) return 0.0;
        return detection.ftcPose.range;
    }
}
