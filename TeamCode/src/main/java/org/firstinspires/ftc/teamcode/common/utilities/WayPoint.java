package org.firstinspires.ftc.teamcode.common.utilities;

import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

public class WayPoint {
    public JewelSwatter.JewelSwatterState jewelSwatterState;
    public Clamp.ClampState clampState;
    public double power;
    public double x, y, z;
    public double waitMillis;

    public WayPoint(double x, double y, double z, double power, Clamp.ClampState clampState, JewelSwatter.JewelSwatterState jewelSwatterState, double waitMillis) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.power = power;
        this.clampState = clampState;
        this.jewelSwatterState = jewelSwatterState;
        this.waitMillis = waitMillis;
    }
}
