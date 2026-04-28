package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DriveSystem {

    private final double WHEEL_DIAMETER_MM = 96;
    private final double ENCODER_TICKS_PER_REV = 537.7;
    private final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    private final double TRACK_WIDTH_MM = 404;

    private final DcMotor leftDrive;
    private final DcMotor rightDrive;

    private final ElapsedTime driveTimer = new ElapsedTime();

    public DriveSystem(HardwareMap hardwareMap, String leftMotorName, String rightMotorName) {

        leftDrive = hardwareMap.get(DcMotor.class, leftMotorName);
        rightDrive = hardwareMap.get(DcMotor.class, rightMotorName);

        // Account for direction of motor relative to front of robot
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reduce coast time when stopping
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

    }

    /**
     *
     * @param forward         From -1 to 1. Full motor power reverse to full forward.
     * @param rotation        From -1 to 1. Pure left turn to right turn.
     * @param speedMultiplier Scales the input values.
     */
    public void driveContinuously(double forward, double rotation, double speedMultiplier) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setPower((forward + rotation) * speedMultiplier);
        rightDrive.setPower((forward - rotation) * speedMultiplier);
    }

    /**
     * Must be called when about to use driveIncrementally or rotateIncrementally with a new target.
     */
    public void stopForNewIncrementalTarget() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * @param speed        From 0-1
     * @param distance     In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds  the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    public boolean driveIncrementally(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) targetPosition);
        rightDrive.setTargetPosition((int) targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if (Math.abs(targetPosition - leftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return driveTimer.seconds() > holdSeconds;
    }

    /**
     * @param speed       From 0-1
     * @param angle       the amount that the robot should rotate
     * @param angleUnit   the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the motors are within tolerance of the target position for more than
     * holdSeconds. False otherwise.
     */
    public boolean rotateIncrementally(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to create the
         * requested angle. We use radians here because it makes the math much easier.
         * Our robot will have rotated one radian when the wheels of the robot have driven
         * 1/2 of the track width of our robot in a circle. This is also the radius of the circle
         * that the robot tracks when it is rotating. So, to find the number of mm that our wheels
         * need to travel, we just need to multiply the requested angle in radians by the radius
         * of our turning circle.
         */
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        /*
         * We need to set the left motor to the inverse of the target so that we rotate instead
         * of driving straight.
         */
        double leftTargetPosition = -(targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        leftDrive.setTargetPosition((int) leftTargetPosition);
        rightDrive.setTargetPosition((int) rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if ((Math.abs(leftTargetPosition - leftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)) {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    public void logStatus(Telemetry telemetry) {
        if (leftDrive.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            // continuous driving
            telemetry.addData("Drive motors", "left (%.2f), right (%.2f)",
                    leftDrive.getPower(), rightDrive.getPower());
        } else {
            // incremental driving
            telemetry.addData("Motor current positions", "left (%d), right (%d)",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            telemetry.addData("Motor target positions", "left (%d), right (%d)",
                    leftDrive.getTargetPosition(), rightDrive.getTargetPosition());
        }

    }
}
