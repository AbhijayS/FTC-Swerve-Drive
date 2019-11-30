package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;

import java.util.ArrayList;

@TeleOp (name = "Opmode: TeleOp")
public class FullTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("TeleOp");
        Robot robot = new Robot(this,robotDebugger);
        robot.requestState(Robot.RobotState.ROBOT_ORIENTED);
        robot.updateAll();

        waitForStart();

        while (opModeIsActive()) {
            robot.updateAll();
        }
    }
}
