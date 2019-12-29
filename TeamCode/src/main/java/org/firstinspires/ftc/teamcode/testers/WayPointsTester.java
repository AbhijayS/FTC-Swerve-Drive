package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.HardStops;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Lift;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

import java.util.ArrayList;

@TeleOp(name = "TestOp: WayPoints")
public class WayPointsTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Waypoint Tester");
        Robot robot = new Robot(this,robotDebugger);

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        WayPoint[] wayPoints = new WayPoint[] {
                new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0,false,0,0, HardStops.HardStopState.STOW,0.01),
                new WayPoint(new Pose(-20,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0,false,0,0, HardStops.HardStopState.STOW,0.01),
                new WayPoint(new Pose(-40,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0,false,0,0, HardStops.HardStopState.STOW,0.01)
        };
        robot.requestState(Robot.RobotState.PATH_FOLLOWING);
        robot.setWayPoints(wayPoints);
        while (opModeIsActive()) {
            robot.updateAll();
        }
        robotDebugger.stopLogging();
    }
}
