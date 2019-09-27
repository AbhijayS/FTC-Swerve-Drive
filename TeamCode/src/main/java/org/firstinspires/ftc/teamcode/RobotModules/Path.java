package org.firstinspires.ftc.teamcode.RobotModules;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;
import org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.Direction;

import java.util.ArrayList;
import java.util.HashMap;

import static java.lang.Double.NaN;

public class Path {

    double[] x;
    double[] y;
    HashMap<Double, Double> X_Y;
    public PolynomialSplineFunction spline;
    private LaguerreSolver laguerreSolver;

    private Direction direction;
    private int splineSegment;

    private SplineInterpolator splineInterpolator;

    public Path(double[] x, double[] y) {
        this.x = x.clone();
        this.y = y.clone();
        splineInterpolator = new SplineInterpolator();
        spline = splineInterpolator.interpolate(this.x, this.y);
        laguerreSolver = new LaguerreSolver();

        direction = Direction.LEFT;
        splineSegment = 1;
    }

    public void setNewPath(double[] x, double[] y){
        this.x = x.clone();
        this.y = y.clone();
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
                    toAppend = toAppend.concat(raw.substring(previous,index)).
                            concat("(x-").
                            concat(offset).
                            concat(")");
                    previous = index+1;
                    index = raw.indexOf('x',previous);
                }
                toAppend = toAppend.concat(raw.substring(previous));
                result = result.concat(toAppend).concat(" || ");
            }
        }catch (Exception e) {
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

        if (CoM.getX() < spline.getKnots()[0])
            return spline.getKnots()[0];

        if (CoM.getX() > spline.getKnots()[spline.getN()])
            return spline.getKnots()[spline.getN()];

        // TODO simplify poly calculations
        boolean found = false;
        int i = 0;
        double trace = oldTrackingPoint.getX();

        while (!found && (splineSegment + i <= spline.getN())) {

            //-----knotA------currentSegment------knotB---//
            int currentSegment = splineSegment + i;
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
            splineSegment += i;
            return trace;
        }
        return NaN;
    }

    // TODO: Minimum Jerk Trajectory (MJT) for start and end splines
    // TODO: MJT for the whole path would be ideal
    private double minimumJerkPower() {
        return 1;
    }

    public Direction getDirection() {
        return direction;
    }
}
