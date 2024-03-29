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
import org.firstinspires.ftc.teamcode.modules.sensory.VuforiaDetector;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_LENGTH;

@Autonomous(name = "Blue Main")
public class BlueFoundation2SkystonePark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Auto_Blue_Main");
        char stonePosition = 'A';
        Robot robot = new Robot(this, robotDebugger);
        VuforiaDetector vd = new VuforiaDetector(hardwareMap);
        stonePosition = vd.getPatternBlue();

        telemetry.update();
        waitForStart();


        // path following stuff
        WayPoint[] wayPoints;
//        switch (stonePosition) {
//            case 'A': {
//                wayPoints = new WayPoint[] {
//                        // pick up first skystone
//                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(-1,-22,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(-2,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // move to foundation
//                        new WayPoint(new Pose(-2,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(40,-27,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
//
//                        // pick up second stone
//                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(78,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(70,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
//                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 2),
//                        new WayPoint(new Pose(10,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 2),
//                        new WayPoint(new Pose(2,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 2),
//                        new WayPoint(new Pose(-2,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 2),
//                        new WayPoint(new Pose(-18,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(-20,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//
//                        // return back to foundation
//                        new WayPoint(new Pose(-20,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.1),
//                        new WayPoint(new Pose(-19,-30,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(-15,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 2),
//                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 2),
//                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 2),
//                        new WayPoint(new Pose(80,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(80,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // park
//                        new WayPoint(new Pose(65,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.HORIZONTAL_LEFT, 0.01),
//                        new WayPoint(new Pose(40,-30,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.HORIZONTAL_LEFT, 0.01),
//                };
//                break;
//            }
//
//            case 'B': {
//                wayPoints = new WayPoint[] {
//                        // pick up first skystone
//                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(3,-14,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // move to foundation
//                        new WayPoint(new Pose(6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(40,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
//
//                        // pick up second stone
//                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(78,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(60,-27,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
//                        new WayPoint(new Pose(50,-27,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
//                        new WayPoint(new Pose(10,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
//                        new WayPoint(new Pose(2,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(-5,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(-15,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // return back to foundation
//                        new WayPoint(new Pose(-15,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // park
//                        new WayPoint(new Pose(65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                };
//                break;
//            }
//
//            default: {
//                wayPoints = new WayPoint[] {
//                        // pick up first skystone
//                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(2,-14,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(4,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // move to foundation
//                        new WayPoint(new Pose(4,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
//
//                        // pick up second stone
//                        new WayPoint(new Pose(70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(68,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(60,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
//                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
//                        new WayPoint(new Pose(10,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
//                        new WayPoint(new Pose(2,-20,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(-2,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(-7,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // return back to foundation
//                        new WayPoint(new Pose(-7,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(-5,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.ADJUST_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(-3,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(10,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//
//                        // park
//                        new WayPoint(new Pose(65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                };
//                break;
//            }
//        }

        switch (stonePosition) {
            case 'A': {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(-2,-18,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(-4,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 500, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(-4,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
//                        new WayPoint(new Pose(-4,-22,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
//                        new WayPoint(new Pose(-4,-22,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(78,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(70,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
                        new WayPoint(new Pose(50,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
                        new WayPoint(new Pose(10,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.1),
                        new WayPoint(new Pose(2,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.1),
                        new WayPoint(new Pose(-2,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.1),
                        new WayPoint(new Pose(-15,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.1),
//                        new WayPoint(new Pose(-21,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
//                        new WayPoint(new Pose(-22,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(-24.5,-23,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(-24.5,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(-24.5,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(-18,-25,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(0,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(40,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // park
                        new WayPoint(new Pose(65,-30,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(40,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                };
                break;
            }

            case 'B': {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(3,-14,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(6,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(40,-27,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(80,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_LEFT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(78,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(60,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
                        new WayPoint(new Pose(10,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
                        new WayPoint(new Pose(2,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-5,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-19,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-19,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-19,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(-19,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // park
                        new WayPoint(new Pose(65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(36,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                };
                break;
            }

            default: {
                wayPoints = new WayPoint[] {
                        // pick up first skystone
                        new WayPoint(new Pose(0,0,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.STOW_ALL, 0.01),
                        new WayPoint(new Pose(0,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_RIGHT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // move to foundation
                        new WayPoint(new Pose(0,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(0,-26,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(40,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
                        new WayPoint(new Pose(60,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),

                        // pick up second stone
                        new WayPoint(new Pose(70,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_HIGH_RIGHT_OPEN, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(68,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(60,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
                        new WayPoint(new Pose(50,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 5),
                        new WayPoint(new Pose(10,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
                        new WayPoint(new Pose(2,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-2,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-12,-24,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.DOCK_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-12,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 200, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(-12,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.POSSESS_LEFT, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // return back to foundation
                        new WayPoint(new Pose(-12,-28,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(10,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 1, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.1),
                        new WayPoint(new Pose(40,-29,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.75),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.RIDE_LOW_LEFT_OPEN, Lift.State.STOW, 4, 1000, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(80,-36,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),

                        // park
                        new WayPoint(new Pose(65,-33,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
                        new WayPoint(new Pose(40,-31,90),1, Clamp.ClampState.PARTIAL, JewelSwatter.JewelSwatterState.STOW_ALL, Lift.State.STOW, 4, 0, false,0,0, HardStops.HardStopState.DRIVE, 0.01),
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
