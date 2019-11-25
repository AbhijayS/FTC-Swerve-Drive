package org.firstinspires.ftc.teamcode.common.utilities;

import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
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
    public final double POWER;
    public final double WAIT_MILLIS;
    public final Pose POSE;
    public final double X, Y, Z;

    public WayPoint(Pose pose, double power, Clamp.ClampState clampState, JewelSwatter.JewelSwatterState jewelSwatterState, double waitMillis) {
        this.POSE = pose;
        this.X = POSE.getX();
        this.Y = POSE.getY();
        this.Z = POSE.getDegrees();
        this.POWER = power;
        this.CLAMP_STATE = clampState;
        this.JEWEL_SWATTER_STATE = jewelSwatterState;
        this.WAIT_MILLIS = waitMillis;
    }
}
