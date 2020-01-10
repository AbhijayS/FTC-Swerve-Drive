package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;

import java.util.ArrayList;

//@TeleOp (name = "TeleOp: Field Oriented")
public class FieldOriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Field Oriented");
        Robot robot = new Robot(this,robotDebugger);
        robot.requestState(Robot.RobotState.FIELD_ORIENTED);
//        robot.updateAll();

        waitForStart();

        while (opModeIsActive()) {
            robot.updateAll();
        }
    }
}
