package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
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
        Robot robot = new Robot(this, robotDebugger);
        String stonePosition = "A";
        WayPoint[] wayPoints;

        switch (stonePosition) {
            case "A": {
                wayPoints = new WayPoint[]{
                        // pick up first skystone
//                        new WayPoint(new Pose(0,0,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,0, true,0,0),
                        new WayPoint(new Pose(0,0,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,0, true,0,0),
                        new WayPoint(new Pose(0,-27,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_LEFT,100, true,0,0),
                        new WayPoint(new Pose(0,-27.5,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,500, false,0,0),
                        // start heading to foundation
                        new WayPoint(new Pose(4,-26,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,0, false,0,0),
                        new WayPoint(new Pose(8,-25,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0),
                        new WayPoint(new Pose(11,-25,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT,0, false,0,0),
                        new WayPoint(new Pose(50,-25,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT,0, false,0,0),
                        new WayPoint(new Pose(52,-25,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL,0, false,0,0),
                        // latch on
                        new WayPoint(new Pose(72,-31,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL,0, true,0,0),
                        new WayPoint(new Pose(72,-35,90),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,200, false,0,-ROBOT_LENGTH/2),
                        // drag back and turn
                        new WayPoint(new Pose(72,-14,180),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,0, false,0,-ROBOT_LENGTH/2),
                        new WayPoint(new Pose(72,-14,180),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,0, false,0,-ROBOT_LENGTH/2),
                };
                break;
            }

            case "B": {
                wayPoints = new WayPoint[]{};
                break;
            }

            default: {
                wayPoints = new WayPoint[]{};
                break;
            }
        }

        robot.requestState(Robot.RobotState.PATH_FOLLOWING);
        robot.setWayPoints(wayPoints);
        robot.updateAll();
        waitForStart();

        while (opModeIsActive()) {
            robot.updateAll();
        }
    }
}
