package org.firstinspires.ftc.teamcode.common.utilities;

import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

public class WayPoint {
    JewelSwatter.JewelSwatterState jewelSwatterState;
    Clamp.ClampState clampState;
    double power;
    double x, y, z;

    public WayPoint(double x, double y, double z, double power, Clamp.ClampState clampState, JewelSwatter.JewelSwatterState jewelSwatterState) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.power = power;
        this.clampState = clampState;
        this.jewelSwatterState = jewelSwatterState;
    }
}
