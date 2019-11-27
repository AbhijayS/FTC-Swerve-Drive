package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Pose;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.WayPoint;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

import java.util.ArrayList;

@Autonomous(name = "Blue Main")
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
                    new WayPoint(new Pose(0,0,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,0, true),
                    new WayPoint(new Pose(0,-27,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.POSSESS_LEFT,800, true),
                    new WayPoint(new Pose(0,-26,90),0.75, Clamp.ClampState.CLAMP, JewelSwatter.JewelSwatterState.DOCK_LEFT,0, false),
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

        robot.setWayPoints(wayPoints);
        robot.updateAll();
        robotDebugger.log();
        waitForStart();

        robot.requestState(Robot.RobotState.PATH_FOLLOWING);

        while (opModeIsActive()) {
            robot.updateAll();
        }
    }
}
