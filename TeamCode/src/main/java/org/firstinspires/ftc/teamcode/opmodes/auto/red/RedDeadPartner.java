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

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.roundTo2DecimalPlaces;

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
        double[] y = {0, -12.5, -24};
        double[] z = {90, 90, 90};

        Path interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        telemetry.update();
        waitForStart();

        //Moves towards quarry
        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            //Opens left clamp
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        // Clamps the sky stone
        clamp.clamp();
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
        Thread.sleep(500);

        x = new double[]{0, 0, 0};
        y = new double[]{-24, -20, -18};
        z = new double[]{90, 90, 90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        //Backs up
        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            //Raises sky stone
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        clamp.noPower();

        x = new double[]{0, 60, 75};
        y = new double[]{-16, -16, -20};
        z = new double[]{90, 90, 90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        //Moves across field to foundation
        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        //Releases sky stone into foundation
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
        Thread.sleep(500);

        x = new double[]{75,0,-15};
        y = new double[]{-20,-20,-20};
        z = new double[]{90,90,90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        //Returns across field to 2nd sky stone
        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

//        x = new double[]{0,-5,-14, -15};
//        y = new double[]{-25,-27,-27, -28};
//        z = new double[]{90,90,90};
//
//        interpolant = new Path(this, x, y, z);
//        swerveDrive.setPath(interpolant);
//        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);
//
//        //Positions in front of 2nd sky stone
//        while (opModeIsActive()) {
//            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
//                break;
//            }
//            jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
//            swerveDrive.swerveKinematics.update();
//            swerveDrive.stanleyPursuit();
//            robotDebugger.log();
//        }

        //Ensures robot is turned to 90 degrees
        while (opModeIsActive()) {
            double yaw = roundTo2DecimalPlaces(swerveDrive.swerveKinematics.getYaw());
            if (Math.abs(yaw) <= 2) {
                break;
            }
            swerveDrive.fod(90,0,swerveDrive.turnPID(90),roundTo2DecimalPlaces(swerveDrive.swerveKinematics.getYaw()));
            swerveDrive.swerveKinematics.update();
            telemetry.update();
        }

        // Clamp the stone
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
        Thread.sleep(500);

        x = new double[]{-15,-15,-15};
        y = new double[]{-28,-29,-30};
        z = new double[]{90,90,90};

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

        // Clamp the stone
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_LEFT);
        Thread.sleep(500);

        //Raise the stone
        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT);
        Thread.sleep(500);

        x = new double[]{-15, 40, 80};
        y = new double[]{-30, -20, -20};
        z = new double[]{90,90,90};

        interpolant = new Path(this, x, y, z);
        swerveDrive.setPath(interpolant);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        while (opModeIsActive()) {
            if (swerveDrive.getState() == SwerveState.PATH_FOLLOWING_COMPLETE) {
                break;
            }
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_LEFT);
        Thread.sleep(500);

        while (opModeIsActive()) {

        }

        robotDebugger.stopLogging();
    }
}
