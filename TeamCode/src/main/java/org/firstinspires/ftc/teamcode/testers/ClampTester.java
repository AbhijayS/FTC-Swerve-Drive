package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;

@TeleOp(name = "TestOp: Clamp")
public class ClampTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Clamp clamp = new Clamp(this);
        Gamepad gamepad = new Gamepad(this);
        gamepad.update();

        waitForStart();

        while (opModeIsActive()) {
            clamp.updateByGamepad(gamepad);
            telemetry.addData("Clamp Status", clamp.getStatus());
            gamepad.update();
            telemetry.update();
        }
    }
}
