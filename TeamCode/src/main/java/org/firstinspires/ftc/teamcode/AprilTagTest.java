package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
@Disabled
public class AprilTagTest extends OpMode {

    AprilTagCam aprilTagCam;
    Alliance sideToCheck = Alliance.RED;;

    @Override
    public void init() {
        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            sideToCheck = Alliance.RED;
        } else if (gamepad1.x) {
            sideToCheck = Alliance.BLUE;
        }

        telemetry.addData("Alliance tag to find", sideToCheck);
    }

    @Override
    public void loop() {
        aprilTagCam.update();

        if (sideToCheck != null) {
            AprilTagDetection detection = aprilTagCam.getDetectedTagOrNull(sideToCheck.aprilTagId);
            if (detection == null) {
                telemetry.addData(sideToCheck.name() + " tag detection", "Tag not detected");
            } else {
                aprilTagCam.printDataForTag(detection);
            }
        }
    }
}
