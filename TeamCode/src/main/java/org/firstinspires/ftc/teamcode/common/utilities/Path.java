package org.firstinspires.ftc.teamcode.common.utilities;

import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;
import org.firstinspires.ftc.teamcode.common.states.PathState;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_STATUS;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.Status.RELEASE;

// TODO: Re-write paths using WayPoints
public class Path {
    public PolynomialSplineFunction spline;
    private SplineInterpolator splineInterpolator;
    private LinkedHashMap<LinkedHashMap<Double, Double>, Direction> paths; // <Coordinates, Direction>
    private LinkedList<LinkedHashMap<Double, Double>> segments;
    private double[] z;
    private LinkedList<WayPoint> wayPoints;
    private Direction direction;
    private int splineSegment;
    private int pathSegment;
    private int zIndex;
    private Pose trackingPose;
    private PathState pathState;
    private Debugger debugger;

    public Path(Debugger debugger, WayPoint[] wayPoints) {
        if (wayPoints.length < 3)
            throw new RuntimeException("Path must include at least 3 points!");

        this.debugger = debugger;
        this.wayPoints.addAll(Arrays.asList(wayPoints));

        this.zIndex = 0;
        int length = wayPoints.length;
        trackingPose = new Pose(
                wayPoints[0].X,
                wayPoints[0].Y,
                wayPoints[0].Z
        );

        // <Coordinates, ID>
        // Later to be converted to <Coordinates, Direction>
        LinkedHashMap<LinkedHashMap<Double, Double>, Integer> raw_paths = new LinkedHashMap<>();

        // List of Coordinates for each path segment
        LinkedHashMap<Double, Double> XY = new LinkedHashMap<>();

        XY.put(wayPoints[0].X, wayPoints[0].Y);
        int comparator = Double.compare(wayPoints[1].X, wayPoints[0].X);

        // Filter out paths based on whether a path segment is directed LEFT, RIGHT, FORWARD, or REVERSE
        for (int i = 1; i < length; i++) {
            if (Double.compare(wayPoints[i].X, wayPoints[i-1].X) == 0) {
                raw_paths.put(XY, comparator);
                XY = new LinkedHashMap<>();
                XY.put(wayPoints[i].X, wayPoints[i].Y);
                comparator = wayPoints[i].Y > wayPoints[i-1].Y ? 2 : -2;
            } else {
                if (Double.compare(wayPoints[i].X, wayPoints[i-1].X) != comparator) {
                    raw_paths.put(XY, comparator);
                    XY = new LinkedHashMap<>();
                    XY.put(wayPoints[i-1].X, wayPoints[i-1].Y);
                    XY.put(wayPoints[i].X, wayPoints[i].Y);
                    comparator = Double.compare(wayPoints[i].X, wayPoints[i-1].X);
                } else {
                    XY.put(wayPoints[i].X, wayPoints[i].Y);
                }
            }
        }
        raw_paths.put(XY, comparator);

        paths = new LinkedHashMap<>();
        segments = new LinkedList<>();
        segments.addAll(raw_paths.keySet());

        for (int i = 0; i < raw_paths.size(); i++) {
            LinkedHashMap<Double, Double> lMap = segments.get(i);
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
        direction = Direction.UNKOWN;
        splineSegment = 1;
        pathSegment = 1;
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
        if (pathSegment > segments.size()) {
            debugger.addData("Path State", pathState.name());
            pathState = PathState.END;
            return;
        }

        LinkedHashMap<Double, Double> currentSegment = segments.get(pathSegment - 1);
        LinkedList<Double> keys = new LinkedList<>(currentSegment.keySet());
        LinkedList<Double> values = new LinkedList<>(currentSegment.values());
        direction = paths.get(currentSegment);

        switch (direction) {
            case RIGHT: {
                if (trackingPose.getX() >= keys.getLast()) {
                    if (pathSegment == segments.size()) {
                        pathState = PathState.END;
                        return;
                    }
                    pathSegment++;
//                    zIndex++;
                    splineSegment = 1;
                    pathState = PathState.SUSTAIN;
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

                double[] x = new double[keys.size()];
                double[] y = new double[keys.size()];
                for (int i = 0; i < currentSegment.size(); i++) {
                    x[i] = keys.get(i);
                    y[i] = values.get(i);
                }
                spline = splineInterpolator.interpolate(x, y);
                double TPX = findTrackingPointRight(CoM);
                double TPY = spline.value(TPX);
                zIndex = Range.clip(zIndex, 0, z.length - 1);
                trackingPose.setPose(TPX, TPY, z[zIndex]);
                pathState = PathState.FOLLOW;
                break;
            }
            case LEFT: {
                if (trackingPose.getX() <= keys.getLast()) {
                    if (pathSegment == segments.size()) {
                        pathState = PathState.END;
                        return;
                    }
                    pathSegment++;
//                    zIndex++;
                    splineSegment = 1;
                    pathState = PathState.SUSTAIN;
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
                Collections.reverse(keys);
                Collections.reverse(values);
                double[] x = new double[keys.size()];
                double[] y = new double[keys.size()];
                for (int i = 0; i < currentSegment.size(); i++) {
                    x[i] = keys.get(i);
                    y[i] = values.get(i);
                }
                spline = splineInterpolator.interpolate(x, y);
                double TPX = findTrackingPointLeft(CoM);
                double TPY = spline.value(TPX);
                zIndex = Range.clip(zIndex, 0, z.length - 1);
                trackingPose.setPose(TPX, TPY, z[zIndex]);
                pathState = PathState.FOLLOW;
                break;
            }
            case FORWARD: {
                LinkedHashMap<Double, Double> prevSegment = segments.get(pathSegment - 2);
                LinkedList<Double> prevValues = new LinkedList<>(prevSegment.values());

                if (trackingPose.getY() >= values.getLast()) {
                    if (pathSegment == segments.size()) {
                        pathState = PathState.END;
                    }
                    pathSegment++;
                    zIndex++;
                    splineSegment = 1;
                    pathState = PathState.SUSTAIN;
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
                zIndex = Range.clip(zIndex, 0, z.length - 1);
                trackingPose.setPose(keys.getFirst(), CoM.getY(), z[zIndex]);
                pathState = PathState.FOLLOW;
                break;
            }
            case REVERSE: {
                LinkedHashMap<Double, Double> prevSegment = segments.get(pathSegment - 2);
                LinkedList<Double> prevValues = new LinkedList<>(prevSegment.values());

                if (trackingPose.getY() <= values.getLast()) {
                    if (pathSegment == segments.size())
                        pathState = PathState.END;
                    pathSegment++;
                    zIndex++;
                    splineSegment = 1;
                    pathState = PathState.SUSTAIN;
                }

//                if (trackingPose.getY() > prevValues.getLast()) {
//                    if (pathSegment == 1)
//                        pathState = PathState.BEGIN;
//                    pathSegment--;
//                    zIndex--;
//                    splineSegment = 1;
//                    pathState = PathState.SUSTAIN;
//                }
                zIndex = Range.clip(zIndex, 0, z.length - 1);
                trackingPose.setPose(keys.getFirst(), CoM.getY(), z[zIndex]);
                pathState = PathState.FOLLOW;
                break;
            }
            default:
                if (pathSegment == segments.size())
                    pathState = PathState.END;
                pathSegment++;
                zIndex++;
                pathState = PathState.PAUSE;
        }

        if (!ROBOT_STATUS.equals(RELEASE)) {
            debugger.addData("Path State", pathState.name());
        }
    }

    private double findTrackingPointRight(Pose CoM) {


        //-----knotA------currentSegment------knotB---//
        int currentSegment = splineSegment;

        if (currentSegment > spline.getN())
            return trackingPose.getX();

        double dTrackingPoint = Math.hypot(CoM.getX() - trackingPose.getX(), CoM.getY() - trackingPose.getY());
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

        Complex[] all = ls.solveAllComplex(fx.getCoefficients(), trackingPose.getX() + shift);
        ArrayList<Double> real = new ArrayList<>();
        ArrayList<Double> realDistances = new ArrayList<>();

        for (Complex c : all) {
            if (c.getImaginary() == 0) {
                double r = c.getReal() - shift;
                if (r > trackingPose.getX() && r < knotB) {
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
            return trackingPose.getX();
        else if (Double.compare(min, dKnotB) == 0) {
            splineSegment++;
            zIndex++;
            return knotB;
        } else
            return real.get(realDistances.indexOf(min));
    }

    private double findTrackingPointLeft(Pose CoM) {


        //-----knotB------currentSegment------knotA---//
        int currentSegment = spline.getN() - splineSegment + 1;

        if (currentSegment > spline.getN())
            return trackingPose.getX();

        double dTrackingPoint = Math.hypot(CoM.getX() - trackingPose.getX(), CoM.getY() - trackingPose.getY());
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

        Complex[] all = ls.solveAllComplex(fx.getCoefficients(), trackingPose.getX() + shift);
        ArrayList<Double> real = new ArrayList<>();
        ArrayList<Double> realDistances = new ArrayList<>();

        for (Complex c : all) {
            if (c.getImaginary() == 0) {
                double r = c.getReal() - shift;
                if (r < trackingPose.getX() && r > knotB) {
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
            return trackingPose.getX();
        else if (Double.compare(min, dKnotB) == 0) {
            splineSegment++;
            zIndex++;
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
        LinkedHashMap<Double, Double> currentSegment = segments.get(0);
        LinkedList<Double> keys = new LinkedList<>(currentSegment.keySet());
        LinkedList<Double> values = new LinkedList<>(currentSegment.values());
        return new Pose(keys.getFirst(), values.getFirst(), 0);
    }

    public PathState getPathState() {
        return pathState;
    }

    public Pose getTrackingPose() {
        return trackingPose;
    }

    public Direction getDirection() {
        return direction;
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
