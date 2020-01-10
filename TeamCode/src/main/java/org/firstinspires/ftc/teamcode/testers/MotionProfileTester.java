package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.ACCEL;
import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.VEL;

//@TeleOp(name = "TestOp: Motion Profile")
public class MotionProfileTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Motion Profile Tester");
        SwerveDrive swerve = new SwerveDrive(this,robotDebugger);
        Stopwatch stopwatch = new Stopwatch();

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double y = swerve.swerveKinematics.getCenterOfMass().getY();
            double power;

            if (t-y <= 1)
                power = 0;
            else
                power = profile(y);

            swerve.fod(90,power,swerve.turnPID(90),swerve.swerveKinematics.getYaw());

            robotDebugger.addData("Loop ms", stopwatch.millis());
            stopwatch.reset();
            stopwatch.start();
            robotDebugger.addData("Y", y);
            robotDebugger.addData("Power", power);
            robotDebugger.addData(VEL.toString(),swerve.swerveKinematics.getVelocity());
            robotDebugger.addData(ACCEL.toString(),swerve.swerveKinematics.getAcceleration());
            robotDebugger.log();
            swerve.swerveKinematics.update();
        }
        robotDebugger.stopLogging();
    }

    public float AMAX = 100f;
    public float DMAX = 25f;
    public float VMAX = 60f;
    public final double C = Math.pow(VMAX,2)/(2*DMAX); // shift x to origin
    public float fDrive = 0.09f;
    public float t = 40f;
    public double profile(double y) {
        // initial power
        double start = 0;
        double end = t;
        double accel = Math.sqrt(2 * AMAX * (y-start)); // acceleration velocity
        double decel = Math.sqrt(Math.pow(VMAX,2) - 2*DMAX*(y+C-end)); // deceleration velocity
        if (accel < decel)
            return Range.scale(Range.clip(accel,0,VMAX),0,VMAX,fDrive,1);
        else
            return Range.scale(Range.clip(decel,0,VMAX),0,VMAX,-fDrive,1);
    }
}
