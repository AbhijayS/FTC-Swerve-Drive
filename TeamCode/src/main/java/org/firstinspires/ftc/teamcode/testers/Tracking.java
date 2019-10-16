package org.firstinspires.ftc.teamcode.testers;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.apache.commons.math3.complex.Complex;
import org.firstinspires.ftc.teamcode.common.utilities.Point;

import java.util.ArrayList;
import java.util.Collections;

public class Tracking {
    static int splineSegment;
    static SplineInterpolator splineInterpolator;
    static PolynomialSplineFunction spline;
    static Point CoM;
    static Point trackingPoint;

    public static void main(String args[]) {
        double x[] = {
                0, 10, 20, 30, 40
        };
        double y[] = {
                0, 20, 0, -20, 0
        };

        // RIGHT
//        CoM.setCoordinates(0, 0);
//        splineInterpolator = new SplineInterpolator();
//        spline = splineInterpolator.interpolate(x, y);
//        System.out.println(str());



//        double inc = 1;
//        double max = 5;
//        for (double i = 0; i < 40; i+=inc) {
//            double s = Math.random() > 0.5 ? 1 : -1;
//            double ry = spline.value(i) + Math.random()*max*s;
//            CoM.setCoordinates(i, ry);
//            double tp = findTrackingPointRight(CoM);
//            System.out.println("(" + CoM.getX() + ", " + CoM.getY() + ")");
//            System.out.println("(" + tp + ", " + spline.value(tp) + ")");
//            trackingPoint.setCoordinates(tp, spline.value(tp));
//        }

        // LEFT
        splineSegment = 4;
        CoM = new Point(40, 0, 0);
        trackingPoint = new Point(40, 0, 0);
        splineInterpolator = new SplineInterpolator();
        spline = splineInterpolator.interpolate(x, y);
        System.out.println(str());

        double inc = -1;
        double max = 5;
        for (double i = 40; i > 0; i+=inc) {
            double s = Math.random() > 0.5 ? 1 : -1;
            double ry = spline.value(i) + Math.random()*max*s;
            CoM.setCoordinates(i, ry);
            double tp = findTrackingPointLeft(CoM);
            System.out.println("(" + CoM.getX() + ", " + CoM.getY() + ")");
            System.out.println("(" + tp + ", " + spline.value(tp) + ")");
            trackingPoint.setCoordinates(tp, spline.value(tp));
        }
    }

    public static double findTrackingPointRight(Point CoM) {

        double dTrackingPoint = Math.hypot(CoM.getX()-trackingPoint.getX(), CoM.getY()-trackingPoint.getY());

        //-----knotA------currentSegment------knotB---//
        int currentSegment = splineSegment;
        Double knotA = spline.getKnots()[currentSegment - 1];
        Double knotB = spline.getKnots()[currentSegment];
        Double shift = -knotA;
        Double dKnotB = Math.hypot(CoM.getX()-knotB, CoM.getY()-spline.value(knotB));

        LaguerreSolver ls = new LaguerreSolver();
        PolynomialFunction ry = new PolynomialFunction(new double[]{CoM.getY()});
        PolynomialFunction rx = new PolynomialFunction(new double[]{-(CoM.getX() + shift), 1});
        PolynomialFunction p = spline.getPolynomials()[currentSegment - 1];
        PolynomialFunction dp = p.polynomialDerivative();

        PolynomialFunction fx = rx.add(dp.multiply(p.subtract(ry))); // Distance formula to optimize

        Complex[] all = ls.solveAllComplex(fx.getCoefficients(), trackingPoint.getX() + shift);
        ArrayList<Double> real = new ArrayList<>();
        ArrayList<Double> realDistances = new ArrayList<>();

        for (Complex c : all) {
            if (c.getImaginary() == 0) {
                double r = c.getReal() - shift;
                if (r > trackingPoint.getX() && r < knotB) {
                    real.add(r);
                    realDistances.add(Math.hypot(CoM.getX()-r, CoM.getY()-spline.value(r)));
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
            return trackingPoint.getX();
        else if (Double.compare(min, dKnotB) == 0) {
            splineSegment++;
            return knotB;
        } else
            return real.get(realDistances.indexOf(min));

    }

    public static double findTrackingPointLeft(Point CoM) {

        double dTrackingPoint = Math.hypot(CoM.getX()-trackingPoint.getX(), CoM.getY()-trackingPoint.getY());

        //-----knotB------currentSegment------knotA---//
        int currentSegment = spline.getN() - splineSegment + 1;
        Double knotB = spline.getKnots()[currentSegment - 1];
        Double knotA = spline.getKnots()[currentSegment];
        Double shift = -knotB;
        Double dKnotB = Math.hypot(CoM.getX()-knotB, CoM.getY()-spline.value(knotB));

        LaguerreSolver ls = new LaguerreSolver();
        PolynomialFunction ry = new PolynomialFunction(new double[]{CoM.getY()});
        PolynomialFunction rx = new PolynomialFunction(new double[]{-(CoM.getX() + shift), 1});
        PolynomialFunction p = spline.getPolynomials()[currentSegment - 1];
        PolynomialFunction dp = p.polynomialDerivative();

        PolynomialFunction fx = rx.add(dp.multiply(p.subtract(ry))); // Distance formula to optimize

        Complex[] all = ls.solveAllComplex(fx.getCoefficients(), trackingPoint.getX() + shift);
        ArrayList<Double> real = new ArrayList<>();
        ArrayList<Double> realDistances = new ArrayList<>();

        for (Complex c : all) {
            if (c.getImaginary() == 0) {
                double r = c.getReal() - shift;
                if (r < trackingPoint.getX() && r > knotB) {
                    real.add(r);
                    realDistances.add(Math.hypot(CoM.getX()-r, CoM.getY()-spline.value(r)));
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
            return trackingPoint.getX();
        else if (Double.compare(min, dKnotB) == 0) {
            splineSegment++;
            return knotB;
        } else
            return real.get(realDistances.indexOf(min));

    }

    public static String str() {
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
