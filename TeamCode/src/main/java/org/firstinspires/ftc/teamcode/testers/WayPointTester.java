package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
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

@TeleOp(name = "TestOp: WayPoint Tester")
public class WayPointTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("WayPoint Tester");

        WayPoint[] wayPoints = {
            // moving backward
//            new WayPoint(new Pose(0,0,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, true),
//            new WayPoint(new Pose(0,12,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, false),
//            new WayPoint(new Pose(5,12,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, false),
//            new WayPoint(new Pose(10,12,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, true),
//            new WayPoint(new Pose(10,0,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, false),

            new WayPoint(new Pose(0,0,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, true,0,0),
            new WayPoint(new Pose(0,12,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0),
            new WayPoint(new Pose(5,12,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0),
            new WayPoint(new Pose(10,12,90),0.5, Clamp.ClampState.COAST, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0)
        };

        Robot robot = new Robot(this, robotDebugger);
        robot.setWayPoints(wayPoints);
        robot.requestState(Robot.RobotState.PATH_FOLLOWING);
        robot.updateAll();


        waitForStart();

        while (opModeIsActive()) {
            robot.updateAll();
        }

        robotDebugger.stopLogging();
    }
}
