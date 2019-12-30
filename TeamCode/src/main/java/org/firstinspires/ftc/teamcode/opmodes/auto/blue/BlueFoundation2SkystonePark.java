package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

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

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_LENGTH;

@TeleOp(name = "Blue Main")
public class BlueFoundation2SkystonePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Blue_Main");
        String stonePosition = "B";
        Robot robot = new Robot(this, robotDebugger);
//        TensorFlowLite tensorFlowLite = new TensorFlowLite(this,.5);
//        tensorFlowLite.activateTfod();
//        robot.updateAll();

        telemetry.update();
        waitForStart();

//        // tensor flow stuff
//        ElapsedTime elapsedTime = new ElapsedTime();
//        while(opModeIsActive() && stonePosition.equals("Unknown")) {
//            tensorFlowLite.twoStone();
//            stonePosition = tensorFlowLite.getPattern();
//            if (!stonePosition.equals("Unknown") || elapsedTime.time() > 3.0) {
//                break;
//            }
//        }
//        tensorFlowLite.shutDownTfod();
//
//        if (stonePosition.equals("A"))
//            stonePosition = "C";
//        else if (stonePosition.equals("C"))
//            stonePosition = "A";

        // path following stuff
        WayPoint[] wayPoints;
        switch (stonePosition) {
            case "A": {
                wayPoints = new WayPoint[]{
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),0.6, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        new WayPoint(new Pose(-0.5,-10,90),0.6, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        new WayPoint(new Pose(-1,-20,90),0.1, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.STOW, 6),
                        new WayPoint(new Pose(-1,-20,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 6),
                        // start heading to foundation
                        new WayPoint(new Pose(-1,-16,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
                        new WayPoint(new Pose(11,-20,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
                        new WayPoint(new Pose(50,-20,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        new WayPoint(new Pose(52,-20,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        new WayPoint(new Pose(60,-25,95),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        new WayPoint(new Pose(69,-25,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        // latch on
                        new WayPoint(new Pose(69,-25,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
                        new WayPoint(new Pose(69,-33,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
                        new WayPoint(new Pose(69,-33,90),0.6, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.STOW, 1),
//                        new WayPoint(new Pose(72,-36,90),0.4, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL,500, false,0,0, HardStops.HardStopState.STOW),
                        // drag back and turn
                        new WayPoint(new Pose(68,-25,90),0.6, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL, Lift.State.STOW, 1, 0, false,0,-ROBOT_LENGTH/2, HardStops.HardStopState.STOW, 6),
                        new WayPoint(new Pose(67,-14,200),0.6, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_ALL, Lift.State.STOW, 1, 3000, false,0,-ROBOT_LENGTH/2, HardStops.HardStopState.PARTIAL_DEPLOY, 6),
                        new WayPoint(new Pose(67,-15,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 800, false,0,0, HardStops.HardStopState.STOW, 1),
                        // return for second stone

                        new WayPoint(new Pose(65,-27,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
                        new WayPoint(new Pose(62,-30,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
                        new WayPoint(new Pose(40,-30,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW, 1),
//                        new WayPoint(new Pose(11,-32,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(0,-32,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.STOW_ALL,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(-5,-23,45),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(-14,-26,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_RIGHT,500, true,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(-14,-36,90),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_RIGHT,100, true,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(-14,-36.5,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        // return to foundation
//                        new WayPoint(new Pose(-10,-33,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(-5,-33,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(0,-33,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW, 6),
//                        new WayPoint(new Pose(65,-33,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN,0, false,0,0, HardStops.HardStopState.STOW, 2),
//                        new WayPoint(new Pose(60,-33,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN,0, false,0,0, HardStops.HardStopState.STOW, 2),
//                        new WayPoint(new Pose(50,-33,180),0.5, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT,0, false,0,0, HardStops.HardStopState.STOW, 2),

                };
                break;
            }

            case "B": {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(3,-14,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(78,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(60,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(10,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),
                        new WayPoint(new Pose(2,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-5,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-15,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(-15,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // park
                        new WayPoint(new Pose(65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                };
                break;
            }

            default: {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(0,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(0,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(1,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.ADJUST_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(3,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(68,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(60,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 5),
                        new WayPoint(new Pose(10,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.75),
                        new WayPoint(new Pose(2,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-2,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(-7,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(-7,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-5,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.ADJUST_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(-3,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(10,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.1),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),

                        // park
                        new WayPoint(new Pose(65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
                        new WayPoint(new Pose(40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DEPLOY, 0.01),
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
