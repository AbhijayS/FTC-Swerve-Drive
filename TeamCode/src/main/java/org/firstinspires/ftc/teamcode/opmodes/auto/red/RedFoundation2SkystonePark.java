package org.firstinspires.ftc.teamcode.opmodes.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.modules.sensory.VuforiaDetector;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_LENGTH;

@TeleOp(name = "Red Main")
public class RedFoundation2SkystonePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Blue_Main");
        char stonePosition = 'A';
        Robot robot = new Robot(this, robotDebugger);
//        VuforiaDetector vd = new VuforiaDetector(hardwareMap);

        telemetry.update();
        waitForStart();

//        stonePosition = vd.getPatternRed();

        // path following stuff
        WayPoint[] wayPoints;
        switch (stonePosition) {
            case 'A': {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(2,-18,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(4,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.DEPLOY, 0.5),

                        // move to foundation
                        new WayPoint(new Pose(4,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.5),
                        new WayPoint(new Pose(-10,-27,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(-80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-78,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 2),
                        new WayPoint(new Pose(-70,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 2),
                        new WayPoint(new Pose(-40,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 2),
//                        new WayPoint(new Pose(-10,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),
//                        new WayPoint(new Pose(-2,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 2),
//                        new WayPoint(new Pose(2,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
//                        new WayPoint(new Pose(20,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(10,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(23,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),

                        // return back to foundation
                        new WayPoint(new Pose(23,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(23,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(24,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(25,-30,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(25,-30,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(10,-27,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
//                        new WayPoint(new Pose(-40,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
//                        new WayPoint(new Pose(-80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
//                        new WayPoint(new Pose(-80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
//
//                        // park
                        new WayPoint(new Pose(-70,-30,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
//                        new WayPoint(new Pose(-65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
//                        new WayPoint(new Pose(-40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                };
                break;
            }

            case 'B': {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-3,-14,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(-6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-80,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(-80,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-78,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-60,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(-50,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(-10,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),
                        new WayPoint(new Pose(-2,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(5,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(15,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(15,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // park
                        new WayPoint(new Pose(-65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                };
                break;
            }

            default: {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(0,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(0,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-1,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.ADJUST_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-3,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(-70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-68,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-60,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(-50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(-10,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),
                        new WayPoint(new Pose(-2,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(2,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(7,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(7,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(5,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.ADJUST_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(3,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-10,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // park
                        new WayPoint(new Pose(-65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                };
                break;
            }
        }

        robot.requestState(Robot.RobotState.PATH_FOLLOWING);
        robot.setWayPoints(wayPoints);

        while (opModeIsActive()) {
            telemetry.addData("Pattern", stonePosition);
            robot.updateAll();
        }

        robotDebugger.stopLogging();
    }
}
