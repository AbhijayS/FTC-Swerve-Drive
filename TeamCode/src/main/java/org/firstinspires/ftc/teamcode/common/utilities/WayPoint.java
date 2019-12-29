package org.firstinspires.ftc.teamcode.common.utilities;

import org.firstinspires.ftc.teamcode.modules.HardStops;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Lift;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

/**
 * Way Pose
 * Describes the x,y coordinates, heading, and power at a particular point on the Path
 * Describes the states of various modules on the robot
 * @see Path
 */
public class WayPoint {
    public final JewelSwatter.JewelSwatterState JEWEL_SWATTER_STATE;
    public final Clamp.ClampState CLAMP_STATE;
    public final HardStops.HardStopState HARD_STOP_STATE;
    public final double POWER;
    public final double WAIT_MILLIS;
    public final Pose POSE;
    public final double X, Y, Z;
    public final boolean enablePID;
    public final double PIVOT_X;
    public final double PIVOT_Y;
    public final double kS;
    public final Lift.State liftState;
    public final int liftHeight;

    public WayPoint(Pose pose, double power, Clamp.ClampState clampState, JewelSwatter.JewelSwatterState jewelSwatterState, Lift.State liftState, int liftHeight, double waitMillis, boolean enablePID, double pivotX, double pivotY, HardStops.HardStopState hardStopState, double kS) {
        this.POSE = pose;
        this.X = POSE.getX();
        this.Y = POSE.getY();
        this.Z = POSE.getDegrees();
        this.POWER = power;
        this.CLAMP_STATE = clampState;
        this.JEWEL_SWATTER_STATE = jewelSwatterState;
        this.HARD_STOP_STATE = hardStopState;
        this.WAIT_MILLIS = waitMillis;
        this.enablePID = enablePID;
        this.PIVOT_X = pivotX;
        this.PIVOT_Y = pivotY;
        this.kS = kS;
        this.liftState = liftState;
        this.liftHeight = liftHeight;
    }

    public boolean equals(WayPoint wayPoint) {
        return (JEWEL_SWATTER_STATE == wayPoint.JEWEL_SWATTER_STATE) &&
                (CLAMP_STATE == wayPoint.CLAMP_STATE) &&
                (POWER == wayPoint.POWER) &&
                (WAIT_MILLIS == wayPoint.WAIT_MILLIS) &&
                (X == wayPoint.X) &&
                (Y == wayPoint.Y) &&
                (Z == wayPoint.Z) &&
                (enablePID == wayPoint.enablePID);
    }
}
