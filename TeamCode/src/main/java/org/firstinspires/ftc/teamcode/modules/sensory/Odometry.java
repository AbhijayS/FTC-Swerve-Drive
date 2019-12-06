package org.firstinspires.ftc.teamcode.modules.sensory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Odometry {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private DcMotor Xencoder, Yencoder;
    private Coordinate origin;
    private double XencoderZero, YencoderZero;
    public ArrayList<Coordinate> save = new ArrayList<Coordinate>();
    private ArrayList<Coordinate> snapshot = new ArrayList<Coordinate>(); // This array List will be a snapshot of the last ten seconds of match position
    public Coordinate current;
    public Coordinate previous;
    private final double CPR = 512;
    private final double wheel = 1.1811; // inches
    private double t1, t2;
    private double velocity; //inches / second



    public void status(String s) {
        telemetry.addLine(s);
        telemetry.update();

    }

    public Odometry(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        Xencoder = hardwareMap.dcMotor.get("Xencode");
        Yencoder = hardwareMap.dcMotor.get("Yencode");
        Xencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Yencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        status("Initialized Encoders");

        XencoderZero = (double) Xencoder.getCurrentPosition();
        YencoderZero = (double) Yencoder.getCurrentPosition();

        status("O position set");

        telemetry.addData("X Origin: ", XencoderZero);
        telemetry.addData("Y Origin: ", YencoderZero);
        origin = new Coordinate(XencoderZero, YencoderZero);
        current = new Coordinate(origin.getX(), origin.getY());
        previous = new Coordinate(0, 0);

        status("Initialized Coordinates");

        t1 = 0;
        t2 = 0;
        velocity = 0;

        status("Kinematic Variables Initialized");
    }

    /**
     * converts inches to ticks
     *
     * @param inch value of inches to convert to ticks
     * @return ticks equivalent to input of inches
     */

    public double inchToTick(double inch) {
        double rotation = inch / (wheel * Math.PI);
        return (rotation * CPR);
    }


    /**
     * converts ticks to inches
     *
     * @param tick value of ticks on encoder to convert
     * @return value of inches equivalent to ticks.
     */
    public double tickToInch(double tick) {
        double rotation = tick / CPR;
        return (rotation * (wheel * Math.PI));
    }

    /**
     * This generates a magnitude of the robots motion in inches. This is used to determine x and y coordinates at any heading that the robot is at
     *
     * @return gives the magnitude of the vector the robot has travelled. Pair this with a heading to find coordinate points
     */

    public double vectorMagnitude() {
        double x = tickToInch(Xencoder.getCurrentPosition());
        double y = tickToInch(Yencoder.getCurrentPosition());
        double vectorSquare = Math.pow(x, 2) + Math.pow(y, 2);
        return Math.sqrt(vectorSquare);
    }

    /**
     * Generates a x coordinate based off of the heading of the robot and magnitude of movement.
     *
     * @param heading feeds in robot heading to generate x coordinate no matter what the heading or rotation is.
     * @return returns the magnitude of the x coordinate in inches.
     */

    public double genX(double heading) {
        return Math.cos(Math.toRadians(heading + 90)) * vectorMagnitude();
    }

    /**
     * Generates a y coordinate based off of the heading of the robot and magnitude of movement.
     *
     * @param heading feeds in robot heading to generate y coordinate no matter what the heading or rotation is
     * @return returns the magnitude of the y coordinate in inches.
     */
    public double genY(double heading) {
        return Math.sin(Math.toRadians(heading + 90)) * vectorMagnitude();
    }


    /**
     * This method sets the current coordinate point as the current position of the robot based off of odometry.
     *
     * @param heading Pass in the current heading of the robot based off of the gyro.
     */
    public void setCurrent(double heading, double time) {
        previous.setXY(current.getX(), current.getY());
        t1 = t2;
        current.setXY(genX(heading), genY(heading));
        t2 = time;
    }

    /**
     * Adds a point of movement to a constantly changing snapshot of robot positioning.
     *
     * @param seconds pass in the current runtime of seconds to determine whether to add a point or not to the snapshot.
     */
    public void addToSnapshot(double seconds) {
        if (seconds % 2 == 0) {
            if (snapshot.size() == 5) {
                snapshot.remove(0);
                snapshot.add(current);
            } else {
                snapshot.add(current);
            }
        }
    }

    /**
     * sets a snapshot of robot movement to be saved so that it can be fed into path following later.
     */
    public void saveSnapshot() {
        save.clear();
        save.addAll(snapshot);
    }

    /**
     * Calculates the distance between the current point and the previous known location. This is used to calculate a velocity of robot based off of odometry readings.
     * @return distance between current and previous point.
     */
    public double calcDistance() {
        double dx = current.getX() - previous.getX();
        double dy = current.getY() - previous.getY();
        double dSquare = Math.pow(dx, 2) + Math.pow(dy, 2);
        return Math.sqrt(dSquare);
    }

    public void calcVelocity() {
        double dP = calcDistance();
        double dT = t2 - t1;
        velocity = dP / dT;
    }

    public double calcDistance(double xCoordinate, double yCoordinate){
        double dx = xCoordinate - current.getX();
        double dy = yCoordinate - current.getY();
        double dSquare = Math.pow(dx, 2) + Math.pow(dy, 2);
        return Math.sqrt(dSquare);
    }

    public double estimateTimeToPoint(double x, double y){
        double distance = calcDistance(x,y);
        return distance / velocity;
    }

    //needs testing to see how it will return angle values.
    public double angularDistToPoint(double heading, double x, double y){
        double adjHeading = heading + 90;
        double dx = x - current.getX();
        double dy = y - current.getY();
        double angleFromZero = Math.toDegrees(Math.atan(y/x));
        double dAngle = angleFromZero - adjHeading;
        while (dAngle >= 270) {dAngle -= 360;}
        while (dAngle <= -90) {dAngle += 360;}
        return dAngle + 90;
    }



}
