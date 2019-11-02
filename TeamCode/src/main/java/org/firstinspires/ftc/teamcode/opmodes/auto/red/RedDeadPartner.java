package org.firstinspires.ftc.teamcode.opmodes.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp(name = "TestOp: Red Dead Partner")
public class RedDeadPartner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Red_Dead_Partner");

        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);
        Clamp clamp = new Clamp(this);
        JewelSwatter jewelSwatter = new JewelSwatter(this.hardwareMap);

        double[] x = {0, 0, 0};
        double[] y = {0, -12.5, -25};
        double[] z = {90, 90, 90};

        Path interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        // Clamp the stone
        clamp.clamp();
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
        Thread.sleep(500);

        x = new double[]{0, 0, 0};
        y = new double[]{-25, -23, -20};
        z = new double[]{90, 90, 90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        x = new double[]{0, 60, 80};
        y = new double[]{-20, -24, -24};
        z = new double[]{90, 0, 90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        // Clamp the stone
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
        Thread.sleep(500);

        x = new double[]{80,60,0};
        y = new double[]{-22,-24,-24};
        z = new double[]{90,0,90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        x = new double[]{0,-5,-10};
        y = new double[]{-24,-24,-24};
        z = new double[]{90,90,90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        while (opModeIsActive()) {
//            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
//                break;
//            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        robotDebugger.stopLogging();
    }
}
