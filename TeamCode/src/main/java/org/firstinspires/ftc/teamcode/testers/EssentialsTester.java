package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.ACCEL;
import static org.firstinspires.ftc.teamcode.common.utilities.Debugger.Marker.VEL;

@TeleOp(name = "TestOp: Vel, Accel")
public class EssentialsTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Robot Essentials 3");
        Gamepad g = new Gamepad(this);
        SwerveDrive swerve = new SwerveDrive(this,robotDebugger);
        swerve.requestState(SwerveState.HUMAN_INPUT);
        ArrayList<Double> vels = new ArrayList<>();
        ArrayList<Double> accels = new ArrayList<>();

        int filter = 0;

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !gamepad1.y) {
            swerve.fod(g);
            swerve.swerveKinematics.update();
            if (filter <= 100)
                filter++;
            else {
                double vel = swerve.swerveKinematics.getVelocity();
                double accel = swerve.swerveKinematics.getAcceleration();
//                vels.add(vel);
//                accels.add(accel);
                robotDebugger.addData(VEL.toString(), vel);
                robotDebugger.addData(ACCEL.toString(), accel);
//                telemetry.addData("Vel", vel);
//                telemetry.addData("Accel", accel);
                robotDebugger.log();
            }
            g.update();
        }
        robotDebugger.stopLogging();

        Collections.sort(vels);
        Collections.sort(accels);
        while (opModeIsActive()) {
            telemetry.addData("Max Vel", vels.get(vels.size()-1));
            telemetry.addData("Max Accel", accels.get(accels.size()-1));
            telemetry.update();
        }
    }
}
