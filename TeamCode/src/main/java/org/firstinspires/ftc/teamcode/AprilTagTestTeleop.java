package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
public class AprilTagTestTeleop extends OpMode {

    AprilTagCam aprilTagCam;

    @Override
    public void init() {
        aprilTagCam = new AprilTagCam(hardwareMap, telemetry, "webcam");
    }

    @Override
    public void loop() {
        aprilTagCam.update();

        TeamSide sideToCheck = null;
        if (gamepad1.b) {
            sideToCheck = TeamSide.RED;
        } else if (gamepad1.x) {
            sideToCheck = TeamSide.BLUE;
        }
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
