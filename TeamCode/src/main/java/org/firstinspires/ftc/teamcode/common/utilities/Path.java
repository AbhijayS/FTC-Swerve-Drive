package org.firstinspires.ftc.teamcode.common.utilities;

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.code.ThrowingInsn;
import org.firstinspires.ftc.teamcode.common.states.PathState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_STATUS;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.Status.RELEASE;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.tolerance;

// TODO: Re-write paths using WayPoints
public class Path {
    // public variables
    public Direction DIRECTION;
    public PathState PATH_STATE;
    public int PATH_SEGMENT;
    public Pose TRACKING_POSE;
//    public WayPoint CURRENT_WAYPOINT;
    public WayPoint SEGMENT_START; // start point of the current path segment
    public WayPoint SEGMENT_END; // end point of the current path segment

    // private variables
    private PolynomialSplineFunction spline;
    private SplineInterpolator splineInterpolator;
    private LinkedHashMap<LinkedList<WayPoint>, Direction> paths; // <Multiple Waypoints, Direction>
    private LinkedList<LinkedList<WayPoint>> segments; // keyset of paths (path segments)
    private WayPoint[] wayPoints; // x, y
    private int splineSegment;
    private Debugger debugger;


    public Path(Debugger debugger, WayPoint[] wayPoints) {
        if (wayPoints.length < 3)
            throw new RuntimeException("Path must include at least 3 points!");

        this.debugger = debugger;

        // waypoints can't be changed after initialization
        this.wayPoints = wayPoints;

        int length = wayPoints.length;
        TRACKING_POSE = new Pose(
                wayPoints[0].X,
                wayPoints[0].Y,
                wayPoints[0].Z
        );

        // <Coordinates, ID>
        // Later to be converted to <Coordinates, Direction>
        LinkedHashMap<LinkedList<WayPoint>, Integer> raw_paths = new LinkedHashMap<>();

        // List of Coordinates for each path segment
        LinkedList<WayPoint> XY = new LinkedList<>();

        XY.add(wayPoints[0]);
        int comparator = Double.compare(wayPoints[1].X, wayPoints[0].X);

        // Filter out paths based on whether a path segment is directed LEFT, RIGHT, FORWARD, or REVERSE
        for (int i = 1; i < length; i++) {
            if (Double.compare(wayPoints[i].X, wayPoints[i-1].X) == 0) {
                raw_paths.put(XY, comparator);
                XY = new LinkedList<>();
                XY.add(wayPoints[i]);
                comparator = wayPoints[i].Y > wayPoints[i-1].Y ? 2 : -2;
            } else {
                if (Double.compare(wayPoints[i].X, wayPoints[i-1].X) != comparator) {
                    raw_paths.put(XY, comparator);
                    XY = new LinkedList<>();
                    XY.add(wayPoints[i-1]);
                    XY.add(wayPoints[i]);
                    comparator = Double.compare(wayPoints[i].X, wayPoints[i-1].X);
                } else {
                    XY.add(wayPoints[i]);
                }
            }
        }
        raw_paths.put(XY, comparator);

        paths = new LinkedHashMap<>();
        segments = new LinkedList<>();
        segments.addAll(raw_paths.keySet());

        for (int i = 0; i < raw_paths.size(); i++) {
            LinkedList<WayPoint> lMap = segments.get(i);
            int direction = raw_paths.get(lMap);
            System.out.println(direction);

            switch (direction) {
                case 1:
                    if (lMap.size() < 3)
                        throw new RuntimeException("Each path segment must contain at least 3 points!");
                    paths.put(lMap, Direction.RIGHT);
                    break;

                case -1:
                    if (lMap.size() < 3)
                        throw new RuntimeException("Each path segment must contain at least 3 points!");
                    paths.put(lMap, Direction.LEFT);
                    break;

                case 2:
                    paths.put(lMap, Direction.FORWARD);
                    break;

                case -2:
                    paths.put(lMap, Direction.REVERSE);
                    break;

                default:
                    // true if the first path segment is a straight line
                    paths.put(lMap, Direction.UNKOWN);
                    break;
            }
        }
        splineInterpolator = new SplineInterpolator();
        spline = splineInterpolator.interpolate(new double[]{0, 1, 2}, new double[]{0, 0, 0});
        DIRECTION = Direction.UNKOWN;
        splineSegment = 1;
        PATH_SEGMENT = 1;
        this.SEGMENT_START = this.wayPoints[0];
        this.SEGMENT_END = this.wayPoints[1];
    }

    /**
     * @param _x
     * @return Curvature equation: y'' / (1+(y'^2))^(3/2)
     */
    public double curvature(double _x) {
        if (spline.isValidPoint(_x)) {
            return Math.abs(((PolynomialSplineFunction) (spline.derivative())).derivative().value(_x)) / Math.pow((1 + Math.pow((spline.derivative().value(_x)), 2)), 1.5);
        }
        return -1;
    }

    public double heading(double _x) {
        if (spline.isValidPoint(_x))
            return Math.toDegrees(Math.atan(spline.derivative().value(_x)));
        return -1;
    }

    public void pathFollowing(Pose CoM) {
        if (PATH_SEGMENT > segments.size()) {
            debugger.addData("Path State", PATH_STATE.name());
            PATH_STATE = PathState.END;
            return;
        }

        LinkedList<WayPoint> currentSegment = segments.get(PATH_SEGMENT - 1);
        DIRECTION = paths.get(currentSegment);
//        LinkedList<Double> keys = new LinkedList<>(currentSegment.keySet());
//        LinkedList<Double> values = new LinkedList<>(currentSegment.values());

        switch (DIRECTION) {
            case RIGHT: {
                if (TRACKING_POSE.getX() >= currentSegment.getLast().X) {
                    SEGMENT_START = currentSegment.getLast();
                    TRACKING_POSE.setPose(currentSegment.getLast().X, currentSegment.getLast().Y, SEGMENT_END.Z);
                    if (PATH_SEGMENT == segments.size()) {
                        PATH_STATE = PathState.END;
                        return;
                    }
                    PATH_SEGMENT++;
                    splineSegment = 1;
                    PATH_STATE = PathState.SUSTAIN;
                    return;
                }

//                if (trackingPose.getX() < keys.getFirst()) {
//                    if (pathSegment == 1) {
//                        pathState = PathState.BEGIN;
//                        return;
//                    }
//                    pathSegment--;
//                    zIndex--;
//                    splineSegment = 1;
//                    pathState = PathState.SUSTAIN;
//                    return;
//                }

                // re-interpolate points
                // only performed once a switch in direction occurs
                // path is usually not being followed whenever a switch occurs
                if (PATH_STATE != PathState.FOLLOW) {
                    double[] x = new double[currentSegment.size()];
                    double[] y = new double[currentSegment.size()];

                    for (int i = 0; i < currentSegment.size(); i++) {
                        x[i] = currentSegment.get(i).X;
                        y[i] = currentSegment.get(i).Y;
                    }
                    spline = splineInterpolator.interpolate(x, y);
                    SEGMENT_END = currentSegment.getLast();
                }

                // update
                SEGMENT_START = currentSegment.get(splineSegment-1);
                double TPX = findTrackingPointRight(CoM);
                double TPY = spline.value(TPX);
                double TPZ = SEGMENT_START.Z;
                TRACKING_POSE.setPose(TPX, TPY, TPZ);
                PATH_STATE = PathState.FOLLOW;
                break;
            }
            case LEFT: {
                if (TRACKING_POSE.getX() <= currentSegment.getLast().X) {
                    SEGMENT_START = currentSegment.getLast();
                    TRACKING_POSE.setPose(currentSegment.getLast().X, currentSegment.getLast().Y, SEGMENT_END.Z);
                    if (PATH_SEGMENT == segments.size()) {
                        PATH_STATE = PathState.END;
                        return;
                    }
                    PATH_SEGMENT++;
                    splineSegment = 1;
                    PATH_STATE = PathState.SUSTAIN;
                    return;
                }

//                if (trackingPose.getX() > keys.getFirst()) {
//                    if (pathSegment == 1) {
//                        pathState = PathState.BEGIN;
//                        return;
//                    }
//                    pathSegment--;
//                    zIndex--;
//                    splineSegment = 1;
//                    pathState = PathState.SUSTAIN;
//                    return;
//                }

                // re-interpolate points
                // only performed once a switch in direction occurs
                // path is usually not being followed whenever a switch occurs
                if (PATH_STATE != PathState.FOLLOW) {
                    LinkedList<WayPoint> copy = new LinkedList<>(currentSegment);
                    Collections.reverse(copy);

                    double[] x = new double[copy.size()];
                    double[] y = new double[copy.size()];
                    for (int i = 0; i < copy.size(); i++) {
                        x[i] = copy.get(i).X;
                        y[i] = copy.get(i).Y;
                    }
                    spline = splineInterpolator.interpolate(x, y);
                    SEGMENT_END = currentSegment.getLast();
                }

                // update
                SEGMENT_START = currentSegment.get(splineSegment-1);
                double TPX = findTrackingPointLeft(CoM);
                double TPY = spline.value(TPX);
                double TPZ = SEGMENT_START.Z;
                TRACKING_POSE.setPose(TPX, TPY, TPZ);
                PATH_STATE = PathState.FOLLOW;
                break;
            }
            case FORWARD: {
//                LinkedList<WayPoint> prevSegment = segments.get(PATH_SEGMENT - 2);
//                LinkedList<Double> prevValues = new LinkedList<>(prevSegment.values());

                if (TRACKING_POSE.getY() >= currentSegment.getLast().Y - tolerance) {
                    SEGMENT_START = currentSegment.getLast();
                    TRACKING_POSE.setPose(currentSegment.getFirst().X, currentSegment.getLast().Y, SEGMENT_END.Z);
                    if (PATH_SEGMENT == segments.size()) {
                        PATH_STATE = PathState.END;
                        return;
                    }
                    PATH_SEGMENT++;
                    splineSegment = 1;
                    PATH_STATE = PathState.SUSTAIN;
                    return;
                }

//                if (trackingPose.getY() < prevValues.getLast()) {
//                    if (pathSegment == 1) {
//                        pathState = PathState.BEGIN;
//                    }
//                    pathSegment--;
//                    zIndex++;
//                    splineSegment = 1;
//                    pathState = PathState.SUSTAIN;
//                }
                TRACKING_POSE.setPose(currentSegment.getFirst().X, CoM.getY(), SEGMENT_START.Z);
                SEGMENT_END = currentSegment.getLast();
                PATH_STATE = PathState.FOLLOW;
                break;
            }
            case REVERSE: {
//                LinkedList<WayPoint> prevSegment = segments.get(PATH_SEGMENT - 2);
//                LinkedList<Double> prevValues = new LinkedList<>(prevSegment.values());

                if (TRACKING_POSE.getY() <= currentSegment.getLast().Y + tolerance) {
                    SEGMENT_START = currentSegment.getLast();
                    TRACKING_POSE.setPose(currentSegment.getFirst().X, currentSegment.getLast().Y, SEGMENT_END.Z);
                    if (PATH_SEGMENT == segments.size()) {
                        PATH_STATE = PathState.END;
                        return;
                    }
                    PATH_SEGMENT++;
                    splineSegment = 1;
                    PATH_STATE = PathState.SUSTAIN;
                    return;
                }

//                if (trackingPose.getY() > prevValues.getLast()) {
//                    if (pathSegment == 1)
//                        pathState = PathState.BEGIN;
//                    pathSegment--;
//                    zIndex--;
//                    splineSegment = 1;
//                    pathState = PathState.SUSTAIN;
//                }
                TRACKING_POSE.setPose(currentSegment.getFirst().X, CoM.getY(), SEGMENT_START.Z);
                SEGMENT_END = currentSegment.getLast();
                PATH_STATE = PathState.FOLLOW;
                break;
            }
            default:
                if (PATH_SEGMENT == segments.size())
                    PATH_STATE = PathState.END;
                PATH_SEGMENT++;
                PATH_STATE = PathState.PAUSE;
        }

        if (!ROBOT_STATUS.equals(RELEASE)) {
            debugger.addData("Path State", PATH_STATE.name());
        }
    }

    private double findTrackingPointRight(Pose CoM) {


        //-----knotA------currentSegment------knotB---//
        int currentSegment = splineSegment;

        if (currentSegment > spline.getN())
            return TRACKING_POSE.getX();

        double dTrackingPoint = Math.hypot(CoM.getX() - TRACKING_POSE.getX(), CoM.getY() - TRACKING_POSE.getY());
        Double knotA = spline.getKnots()[currentSegment - 1];
        Double knotB = spline.getKnots()[currentSegment];
        Double shift = -knotA;
        Double dKnotB = Math.hypot(CoM.getX() - knotB, CoM.getY() - spline.value(knotB));

        LaguerreSolver ls = new LaguerreSolver();
        PolynomialFunction ry = new PolynomialFunction(new double[]{CoM.getY()});
        PolynomialFunction rx = new PolynomialFunction(new double[]{-(CoM.getX() + shift), 1});
        PolynomialFunction p = spline.getPolynomials()[currentSegment - 1];
        PolynomialFunction dp = p.polynomialDerivative();

        PolynomialFunction fx = rx.add(dp.multiply(p.subtract(ry))); // Distance formula to optimize

        Complex[] all = ls.solveAllComplex(fx.getCoefficients(), TRACKING_POSE.getX() + shift);
        ArrayList<Double> real = new ArrayList<>();
        ArrayList<Double> realDistances = new ArrayList<>();

        for (Complex c : all) {
            if (c.getImaginary() == 0) {
                double r = c.getReal() - shift;
                if (r > TRACKING_POSE.getX() && r < knotB) {
                    real.add(r);
                    realDistances.add(Math.hypot(CoM.getX() - r, CoM.getY() - spline.value(r)));
                }
            }
        }

        Collections.sort(realDistances);

        double min = Math.min(dTrackingPoint, dKnotB);
        for (Double d : realDistances) {
            if (d < min)
                min = d;
        }

        if (Double.compare(min, dTrackingPoint) == 0)
            return TRACKING_POSE.getX();
        else if (Double.compare(min, dKnotB) == 0) {
            splineSegment++;
            return knotB;
        } else
            return real.get(realDistances.indexOf(min));
    }

    private double findTrackingPointLeft(Pose CoM) {


        //-----knotB------currentSegment------knotA---//
        int currentSegment = spline.getN() - splineSegment + 1;

        if (currentSegment > spline.getN())
            return TRACKING_POSE.getX();

        double dTrackingPoint = Math.hypot(CoM.getX() - TRACKING_POSE.getX(), CoM.getY() - TRACKING_POSE.getY());
        Double knotB = spline.getKnots()[currentSegment - 1];
        Double knotA = spline.getKnots()[currentSegment];
        Double shift = -knotB;
        Double dKnotB = Math.hypot(CoM.getX() - knotB, CoM.getY() - spline.value(knotB));

        LaguerreSolver ls = new LaguerreSolver();
        PolynomialFunction ry = new PolynomialFunction(new double[]{CoM.getY()});
        PolynomialFunction rx = new PolynomialFunction(new double[]{-(CoM.getX() + shift), 1});
        PolynomialFunction p = spline.getPolynomials()[currentSegment - 1];
        PolynomialFunction dp = p.polynomialDerivative();

        PolynomialFunction fx = rx.add(dp.multiply(p.subtract(ry))); // Distance formula to optimize

        Complex[] all = ls.solveAllComplex(fx.getCoefficients(), TRACKING_POSE.getX() + shift);
        ArrayList<Double> real = new ArrayList<>();
        ArrayList<Double> realDistances = new ArrayList<>();

        for (Complex c : all) {
            if (c.getImaginary() == 0) {
                double r = c.getReal() - shift;
                if (r < TRACKING_POSE.getX() && r > knotB) {
                    real.add(r);
                    realDistances.add(Math.hypot(CoM.getX() - r, CoM.getY() - spline.value(r)));
                }
            }
        }

        Collections.sort(realDistances);

        double min = Math.min(dTrackingPoint, dKnotB);
        for (Double d : realDistances) {
            if (d < min)
                min = d;
        }

        if (Double.compare(min, dTrackingPoint) == 0)
            return TRACKING_POSE.getX();
        else if (Double.compare(min, dKnotB) == 0) {
            splineSegment++;
            return knotB;
        } else
            return real.get(realDistances.indexOf(min));

    }

    // TODO: Minimum Jerk Trajectory (MJT) for start and end splines
    // TODO: MJT for the whole path would be ideal
    private double minimumJerkPower() {
        return 1;
    }

    public Pose getStartLocation() {
        LinkedList<WayPoint> currentSegment = segments.getFirst();
        return new Pose(currentSegment.getFirst().X, currentSegment.getFirst().Y, currentSegment.getFirst().Z);
    }

    @Override
    public String toString() {
        String result = " || ";
        try {
            for (int i = 0; i < spline.getPolynomials().length; i++) {
                PolynomialFunction p = spline.getPolynomials()[i];
                String offset = Double.toString(spline.getKnots()[i]);
                String raw = p.toString();

                int previous = 0;
                int index = raw.indexOf('x');
                String toAppend = "";
                while (index != -1) {
                    toAppend = toAppend.concat(raw.substring(previous, index)).
                            concat("(x-").
                            concat(offset).
                            concat(")");
                    previous = index + 1;
                    index = raw.indexOf('x', previous);
                }
                toAppend = toAppend.concat(raw.substring(previous));
                result = result.concat(toAppend).concat(" || ");
            }
        } catch (Exception e) {
            throw new RuntimeException("Path toString() method has issues: " + e.getMessage());
        }
        return result;
    }
}
