package org.firstinspires.ftc.teamcode;

public enum ShootingPosition {
    AGAINST_GOAL, ACROSS_FIELD;

    public ShootingPosition getOpposite() {
        if (this == AGAINST_GOAL)
            return ACROSS_FIELD;
        else
            return AGAINST_GOAL;
    }
}
