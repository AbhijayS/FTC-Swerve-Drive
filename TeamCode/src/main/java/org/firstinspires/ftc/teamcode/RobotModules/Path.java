package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedList;

import static java.lang.Double.NaN;

public class Path {
    public PolynomialSplineFunction spline;
    private SplineInterpolator splineInterpolator;
    private LinkedHashMap<LinkedHashMap<Double, Double>, UniversalConstants.Direction> paths;
    private LinkedList<LinkedHashMap<Double, Double>> segments;
    private LinearOpMode linearOpMode;
    private UniversalConstants.Direction direction;
    private int polySegment;
    private int splineSegment;


    public Path(LinearOpMode linearOpMode, double[] x, double[] y) {
        this.linearOpMode = linearOpMode;
        if (x.length != y.length)
            throw new RuntimeException("Invalid arguments to the path!");

        if (x.length < 3)
            throw new RuntimeException("Path must include at least 3 points!");

        int length = x.length;

        LinkedHashMap<LinkedHashMap<Double, Double>, Integer> raw_paths = new LinkedHashMap<>();
        LinkedHashMap<Double, Double> XY = new LinkedHashMap<>();
        XY.put(x[0], y[0]);
        int comparator = Double.compare(x[1], x[0]);

        for (int i = 1; i < length; i++) {
            if (Double.compare(x[i], x[i - 1]) == 0) {
                raw_paths.put(XY, comparator);
                XY = new LinkedHashMap<>();
                XY.put(x[i], y[i]);
                comparator = y[i] > y[i - 1] ? 2 : -2;
            } else {
                if (Double.compare(x[i], x[i - 1]) != comparator) {
                    raw_paths.put(XY, comparator);
                    XY = new LinkedHashMap<>();
                    XY.put(x[i - 1], y[i - 1]);
                    XY.put(x[i], y[i]);
                    comparator = Double.compare(x[i], x[i - 1]);
                } else {
                    XY.put(x[i], y[i]);
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
                    paths.put(lMap, UniversalConstants.Direction.RIGHT);
                    break;

                case -1:
                    if (lMap.size() < 3)
                        throw new RuntimeException("Each path segment must contain at least 3 points!");
                    paths.put(lMap, UniversalConstants.Direction.LEFT);
                    break;

                case 2:
                    paths.put(lMap, UniversalConstants.Direction.FORWARD);
                    break;

                case -2:
                    paths.put(lMap, UniversalConstants.Direction.REVERSE);
                    break;

                default:
                    // true if the first path segment is a straight line
                    paths.put(lMap, UniversalConstants.Direction.UNKOWN);
                    break;
            }
        }
        splineInterpolator = new SplineInterpolator();
        spline = splineInterpolator.interpolate(new double[]{0,1,2}, new double[]{0,0,0});
        direction = UniversalConstants.Direction.UNKOWN;
        polySegment = 1;
        splineSegment = 1;
        linearOpMode.telemetry.addLine(paths.toString());
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


    /*
        * Assumes robot is always moving to the right (positive x direction)
        *
        1. check to see if robot's x position is within bounds
        2. find min distance from the robot
            * knotA
            * laguerre for current segment
            * knotB

     */
    public double findTrackingPoint(Point oldTrackingPoint, Point CoM) {

//        if (CoM.getX() < spline.getKnots()[0])
//            return spline.getKnots()[0];
//
//        if (CoM.getX() > spline.getKnots()[spline.getN()])
//            return spline.getKnots()[spline.getN()];

        // TODO simplify poly calculations
        boolean found = false;
        int i = 0;
        double trace = oldTrackingPoint.getX();

        while (!found && (polySegment + i <= spline.getN())) {

            //-----knotA------currentSegment------knotB---//
            int currentSegment = polySegment + i;
            double knotA = spline.getKnots()[currentSegment - 1];
            double knotB = spline.getKnots()[currentSegment];
            double shift = -knotA;

            LaguerreSolver ls = new LaguerreSolver();
            PolynomialFunction ry = new PolynomialFunction(new double[]{CoM.getY()});
            PolynomialFunction rx = new PolynomialFunction(new double[]{-(CoM.getX() + shift), 1});
            PolynomialFunction p = spline.getPolynomials()[currentSegment - 1];
            PolynomialFunction dp = p.polynomialDerivative();

            PolynomialFunction fx = rx.add(dp.multiply(p.subtract(ry))); // Distance formula to optimize

            Complex[] all = ls.solveAllComplex(fx.getCoefficients(), trace + shift);
            ArrayList<Double> real = new ArrayList<>();


            for (Complex c : all) {
                if (c.getImaginary() == 0) {
                    real.add(c.getReal());
                }
            }

            if (all.length == 0 || real.size() == 0) {
                break;
            }

            for (Double r : real) {
                double prospect = r - shift;

                if (prospect < trace - 0.01) {
                    continue;
                }

                if (prospect <= knotB) {
                    found = true;
                    trace = prospect;
                    break;
                } else {
                    trace = knotB;
                    i++;
                    break;
                }
            }
        }

        if (found) {
            polySegment += i;
            return trace;
        }
        return NaN;
    }

    // TODO: Minimum Jerk Trajectory (MJT) for start and end splines
    // TODO: MJT for the whole path would be ideal
    private double minimumJerkPower() {
        return 1;
    }

    public UniversalConstants.Direction getDirection() {
        return direction;
    }

    /**
     * Path following - just for rightward movement
     * get robot pose
     * check if robot is within lower and upper bounds
     * if (direction == right)
     * return tracking point
     * else
     * return NaN
     */
    public double pathFollowing(Point oldTrackingPoint, Point CoM) {
        LinkedHashMap<Double, Double> currentSegment = segments.get(splineSegment - 1);
        LinkedList<Double> keys = new LinkedList<>(currentSegment.keySet());
        LinkedList<Double> values = new LinkedList<>(currentSegment.values());

        double[] x = new double[keys.size()];
        double[] y = new double[keys.size()];
        for (int i = 0; i < currentSegment.size(); i++) {
            x[i] = keys.get(i);
            y[i] = values.get(i);
        }
        linearOpMode.telemetry.addLine("Segment: " + Arrays.toString(x));
        if (CoM.getX() < keys.getFirst())
            return keys.getFirst();

        if (CoM.getX() > keys.getLast())
            return keys.getLast();

        if (paths.get(currentSegment) == UniversalConstants.Direction.RIGHT) {
            direction = UniversalConstants.Direction.RIGHT;
            spline = splineInterpolator.interpolate(x, y);
            return findTrackingPoint(oldTrackingPoint, CoM);
        }
        return NaN;
    }
}
