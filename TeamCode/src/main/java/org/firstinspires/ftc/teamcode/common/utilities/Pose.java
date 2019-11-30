package org.firstinspires.ftc.teamcode.common.utilities;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clipAngle;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.decimalFormat;

/*
 * TODO This class should be renamed to Pose2D to better describe its purpose
 */
public class Pose {

    private double x, y, heading, originalHeading;

    // heading is in DEGREES
    // TODO Make an Enum to specify units (DEGREES, RADIANS, STRAIGHT, RIGHT, ETC.)

    public Pose() {
        x = 0;
        y = 0;
        heading = 90;
        originalHeading = 90;
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.originalHeading = this.heading;
    }

    public void setHeading(double degrees) {
        heading = clipAngle(degrees);
    }

    public void setCoordinates(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void setPose(double x, double y, double degrees) {
        this.x = x;
        this.y = y;
        heading = clipAngle(degrees);
    }

    /**
     * @param distance Moves the point by the specified distance in the current direction in the global coordinate system
     * @see #getDegrees()
     */
    public void move(double distance) {
        x += distance * Math.cos(Math.toRadians(heading));
        y += distance * Math.sin(Math.toRadians(heading));
    }

    /**
     * @param distance  Self-explanatory
     * @param direction Self-explanatory
     *                  IMPORTANT this method will NOT update the heading of this point
     *                  <p>
     *                  Moves the point by the specified distance in the specified direction in the global coordinate system
     */
    public void move(double distance, double direction) {
        x += distance * Math.cos(Math.toRadians(direction));
        y += distance * Math.sin(Math.toRadians(direction));

    }

    public void translateBy(double x, double y) {
        this.x += x;
        this.y += y;
    }

    public double getDegrees() {
        return heading;
    }

    public double getRadians() {
        return Math.toRadians(heading);
    }

    public double getOriginalHeading() {
        return originalHeading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    @Override
    public String toString() {

        return "(" + decimalFormat.format(x) + ", " + decimalFormat.format(y) + ")";
    }
}
