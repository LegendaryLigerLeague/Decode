package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class RackResetMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();
        RackSystem rackSystem = new RackSystem(hardwareMap, "rack_control", "rack_button");
        //noinspection StatementWithEmptyBody
        while (!isStopRequested() && !rackSystem.cycleToLoadingPosition()) {
        }
    }
}
