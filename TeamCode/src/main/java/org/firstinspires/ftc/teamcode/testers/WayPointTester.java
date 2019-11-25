package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.states.SwerveState;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Path;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp(name = "TestOp: Heading follower Right")
public class WayPointTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("WayPoint Tester");

        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);
        WayPoint[] wayPoints = {
//                // moving right
//                new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(12,0,0),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(24,0,90),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(36,0,45),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(48,0,180),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//
//                // moving left
//                new WayPoint(new Pose(48,0,180),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(36,0,45),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(24,0,90),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(12,0,0),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
//                new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),

                // moving forward
                new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
                new WayPoint(new Pose(0,12,0),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
                new WayPoint(new Pose(0,24,45),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),

                // moving forward
                new WayPoint(new Pose(0,24,45),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
                new WayPoint(new Pose(0,12,0),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
                new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0),
        };

        Path interpolant = new Path(robotDebugger, wayPoints);
        swerveDrive.setPath(interpolant, .3);
        swerveDrive.requestState(SwerveState.PATH_FOLLOWING);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            swerveDrive.swerveKinematics.update();
            swerveDrive.stanleyPursuit();
            robotDebugger.log();
        }

        robotDebugger.stopLogging();
    }
}
