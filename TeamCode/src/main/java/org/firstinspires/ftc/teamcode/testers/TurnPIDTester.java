package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

//@TeleOp(name = "DEBUGGING: Turn PID")
public class TurnPIDTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double target = -90;
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Turn PID");
        SwerveDrive swerve = new SwerveDrive(this,robotDebugger);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            swerve.fod(90,0,swerve.turnPID(target),swerve.swerveKinematics.getYaw());
            swerve.swerveKinematics.update();
            telemetry.addData("Heading", swerve.swerveKinematics.getCenterOfMass().getDegrees());
            telemetry.update();
        }
    }
}
