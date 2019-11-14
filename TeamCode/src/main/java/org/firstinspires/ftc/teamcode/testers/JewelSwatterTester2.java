package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

@TeleOp(name = "TestOp: Clamp")
public class JewelSwatterTester2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JewelSwatter JewelSwatter = new JewelSwatter(this);


        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                // cycles through states
                JewelSwatter.update();
                while (gamepad1.x) {
                    // wait for release
                }
            }
            telemetry.update();
        }
    }
}
