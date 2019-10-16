package org.firstinspires.ftc.teamcode.testers;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kS;

public class ApproachAngle {
    public static void main(String args[]) {
        double[] velocity = {0, 0.5,5}; // fps
        double e = 5; //in

        System.out.printf("Error %f\n", e);
        for (double i = velocity[0]; i <= velocity[2]; i+=0.5) {
            System.out.printf("\tVelocity %f => Heading %f\n", i, calculate(i*12, e));
        }
    }

    static double calculate(double vel, double err) {
        return Math.toDegrees(Math.atan2(err, vel/ kS));
    }
}
