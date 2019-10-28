package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.jewelSwatter.arm;

@TeleOp (name = "armTester")

public class armTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        Boolean hi = false;
        double claw_pos = 0;
        double arm_pos = 0;
        arm arm1 = new arm(this);
        waitForStart();
        while (opModeIsActive()) {
            //sets the positions if you press x
            if (gamepad2.x) {
                claw_pos = 180;
                arm1.setPositions(arm_pos, claw_pos);//arm = 180 claw = 180
                arm1.updateTelemetry();
            }
            //sets the positions if you press y
            if (gamepad2.y) {
                claw_pos = 0;
                arm_pos = 0;
                arm1.setPositions(arm_pos, claw_pos); //arm = 90 claw = 0
                arm1.updateTelemetry();

            }
            if (gamepad2.a) {
                claw_pos = 180;
                arm_pos = 0;
                arm1.setPositions(arm_pos, claw_pos);
                arm1.updateTelemetry();
            }
        }
    }
}
