package org.firstinspires.ftc.teamcode.modules.jewelSwatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "armTester")

public class armTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        arm arm1 = new arm(this);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x) {
                arm1.setPositions(90, 45);
                arm1.updateTelemetry();
            }
            if (gamepad2.y) {
                arm1.setPositions(180, 90);
                arm1.updateTelemetry();

            }
        }
    }
}
