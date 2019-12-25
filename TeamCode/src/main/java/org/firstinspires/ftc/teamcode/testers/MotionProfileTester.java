package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp(name = "TestOp: Motion Profile")
public class MotionProfileTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Motion Profile Tester");
        SwerveDrive swerve = new SwerveDrive(this,robotDebugger);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y = swerve.swerveKinematics.getCenterOfMass().getY();
            double power = profile(y);
            swerve.fod(90,power,swerve.turnPID(90),swerve.swerveKinematics.getYaw());
            telemetry.addData("Y", y);
            telemetry.addData("Power",power);
            telemetry.update();
            swerve.swerveKinematics.update();
        }
    }

    public double aMax = 40;
    public double vMax = 40;
    public double f = 0.09;
    public double t = 40;
    public double profile(double x) {
        if (x < 0.05)
            return 0.15;
        double vel = Range.clip(Math.sqrt(2 * aMax * x),0,vMax);
        double max = maxAllowed(x,t);
        if (vel < max)
            return Range.scale(vel,0,vMax,f,1);
        else
            return Range.scale(max,0,vMax,f,1);
    }

    public double maxAllowed(double x, double target) {
        return Range.clip(Math.sqrt(Math.abs(Math.pow(vMax,2) - 2*aMax*(x-target))),0,vMax);
    }
}
