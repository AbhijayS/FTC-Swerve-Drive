package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.HardStops;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_LENGTH;

@TeleOp(name = "Blue Main")
public class BlueFoundation2SkystonePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Blue_Main");
        String stonePosition = "C";
        Robot robot = new Robot(this, robotDebugger);
        robot.updateAll();

        waitForStart();

        WayPoint[] wayPoints;
        switch (stonePosition) {
            case "A": {
                wayPoints = new WayPoint[]{
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,500, true,0,0, HardStops.HardStopState.DEPLOY),
                        new WayPoint(new Pose(0,-27,90),0.1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_LEFT,100, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(0,-27.5,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        // start heading to foundation
                        new WayPoint(new Pose(0,-23,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(11,-27,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(50,-27,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.DEPLOY),
                        new WayPoint(new Pose(52,-27,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL,0, false,0,0, HardStops.HardStopState.DEPLOY),
                        // latch on
                        new WayPoint(new Pose(68,-33,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL,0, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(68,-34,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(68,-36,90),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,500, false,0,0, HardStops.HardStopState.STOW),
                        // drag back and turn
                        new WayPoint(new Pose(67,-25,90),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,0, false,0,-ROBOT_LENGTH/2, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(66,-18,180),0.3, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,5000, false,0,-ROBOT_LENGTH/2, HardStops.HardStopState.DEPLOY),
                        new WayPoint(new Pose(66,-20,180),0.6, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.DEPLOY),
                        // return for second stone
                        new WayPoint(new Pose(50,-28,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(11,-28,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(0,-23,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-22,-28,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,500, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-22,-35,90),0.1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_LEFT,500, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-22,-35.5,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-10,-30,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(0,-34,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(75,-34,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,500, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(73,-34,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(60,-34,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(50,-34,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                };
                break;
            }

            case "B": {
                wayPoints = new WayPoint[]{};
                break;
            }

            default: {
                wayPoints = new WayPoint[]{
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_RIGHT,500, true,0,0, HardStops.HardStopState.PARTIAL_DEPLOY),
                        new WayPoint(new Pose(0,-26,90),0.1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_RIGHT,100, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(0,-26.5,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        // start heading to foundation
                        new WayPoint(new Pose(0,-23,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(11,-27,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(50,-27,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY),
                        new WayPoint(new Pose(52,-27,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL,0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY),
                        // latch on
                        new WayPoint(new Pose(68,-33,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL,0, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(68,-34,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(68,-36,90),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,500, false,0,0, HardStops.HardStopState.STOW),
                        // drag back and turn
                        new WayPoint(new Pose(67,-25,90),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,0, false,0,-ROBOT_LENGTH/2, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(66,-18,180),0.3, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,5000, false,0,-ROBOT_LENGTH/2, HardStops.HardStopState.PARTIAL_DEPLOY),
                        new WayPoint(new Pose(66,-20,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY),
                        // return for second stone
                        new WayPoint(new Pose(50,-30,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(11,-30,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-5,-23,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-22,-28,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_RIGHT,500, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-22,-36,90),0.2, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_RIGHT,100, true,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-22,-36.5,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(-10,-34,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(0,-34,180),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(75,-34,180),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_RIGHT,500, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(73,-34,180),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(60,-34,180),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                        new WayPoint(new Pose(50,-34,180),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW),
                };
                break;
            }
        }

        robot.requestState(Robot.RobotState.PATH_FOLLOWING);
        robot.setWayPoints(wayPoints);

        while (opModeIsActive()) {
            robot.updateAll();
        }
    }
}
