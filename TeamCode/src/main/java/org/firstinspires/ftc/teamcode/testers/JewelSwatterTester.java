package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

@TeleOp(name = "TestOp: Jewel Swatter")
public class JewelSwatterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JewelSwatter jewelSwatter = new JewelSwatter(this.hardwareMap);

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

            telemetry.addData("STATUS", jewelSwatter.getStatus());
            jewelSwatter.update();
            telemetry.update();
        }
    }
}
