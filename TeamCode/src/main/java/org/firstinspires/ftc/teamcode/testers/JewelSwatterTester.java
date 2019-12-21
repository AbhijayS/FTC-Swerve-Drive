package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelArm;
import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

import java.util.ArrayList;

@TeleOp(name = "TestOp: Jewel Swatter")
public class JewelSwatterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("Jewel Swatter");
        JewelSwatter jewelSwatter = new JewelSwatter(hardwareMap,robotDebugger);
        jewelSwatter.update();

        waitForStart();

        while (opModeIsActive()) {
            //sets the positions if you press x
            if (gamepad2.x) {
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.DOCK_ALL);
            }
            //sets the positions if you press y
            if (gamepad2.y) {
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.POSSESS_ALL);
            }
            if (gamepad2.a) {
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.STOW_ALL);
            }

            if (gamepad2.dpad_up){
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_HIGH_ALL);
            }

            if (gamepad2.dpad_down){
                jewelSwatter.requestState(JewelSwatter.JewelSwatterState.RIDE_LOW_ALL);
            }

            jewelSwatter.update();
            robotDebugger.log();
        }
    }
}
