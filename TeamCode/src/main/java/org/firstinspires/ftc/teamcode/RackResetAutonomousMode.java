package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RackResetAutonomousMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        RackSystem rackSystem = new RackSystem(hardwareMap,"rack_control", "rack_button");
        waitForStart();
        telemetry.addData("Press A to stop","");
        telemetry.update();
        while(!isStopRequested() && !gamepad1.aWasPressed()) {
            rackSystem.setServoToRackLoadingPosition();
        }
    }
}
