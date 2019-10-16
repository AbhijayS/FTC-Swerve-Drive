package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp(name = "TestOP: Heading")
public class MaintainHeading extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("stanley_kin");

        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);

        double setAngle = -180;
        double kP = 0.2/90;
        double tolerance = 0.5;
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            swerveDrive.swerveKinematics.update();
            double heading = swerveDrive.swerveKinematics.getCenterOfMass().getDegrees();
            double err = heading - setAngle;
            err += (err> 180) ? -360 : (err < -180) ? 360 : 0;
            double power = Math.abs(err) <= tolerance ? 0 : err * kP;

            swerveDrive.fod(90,0,power,heading);
            telemetry.addData("Power", power);
            telemetry.addData("Angle", heading);
            telemetry.update();
        }
    }
}
