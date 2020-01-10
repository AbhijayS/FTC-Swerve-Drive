package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

@Autonomous(name = "Just Drive Forward")
public class BlueJustPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Just_Park");
        Robot robot = new Robot(this, robotDebugger);

        telemetry.update();
        waitForStart();

        WayPoint wp[] = new WayPoint[] {
                new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                new WayPoint(new Pose(0,15,90),1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                new WayPoint(new Pose(0,30,90),1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
        };

        robot.requestState(Robot.RobotState.PATH_FOLLOWING);
        robot.setWayPoints(wp);

        while (opModeIsActive()) {
            robot.updateAll();
        }

        robotDebugger.stopLogging();
    }
}
