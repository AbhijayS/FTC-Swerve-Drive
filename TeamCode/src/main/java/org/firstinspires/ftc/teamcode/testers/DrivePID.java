package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp(name = "TestOp: Drive PID")
public class DrivePID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("WayPoint Tester");
        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);
        swerveDrive.swivel(-90);


        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

//        swerveDrive.reverseMotorDirections();
        swerveDrive.movePID(-6, 0.25);
        swerveDrive.enablePID();

        while (opModeIsActive()) {
            telemetry.addLine("Moving to -6 inches");
            swerveDrive.swerveKinematics.update();
            robotDebugger.log();
        }
    }
}
