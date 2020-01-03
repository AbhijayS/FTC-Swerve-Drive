package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.HardStops;

@TeleOp(group = "TestOp", name = "Hard Stop Tester")
public class HardStopTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g = new Gamepad(this);
        HardStops stops = new HardStops(hardwareMap);

        telemetry.addLine("Operator get ready!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up)
                stops.requestState(HardStops.HardStopState.STOW_ALL);
            if (gamepad2.dpad_left)
                stops.requestState(HardStops.HardStopState.HORIZONTAL_LEFT);
            if (gamepad2.dpad_down)
                stops.requestState(HardStops.HardStopState.PROTECT_LEFT);
            if (gamepad2.y)
                stops.requestState(HardStops.HardStopState.STOW_ALL);
            if (gamepad2.b)
                stops.requestState(HardStops.HardStopState.HORIZONTAL_RIGHT);
            if (gamepad2.a)
                stops.requestState(HardStops.HardStopState.PROTECT_RIGHT);
            stops.update();

            telemetry.addData("Stop State: ", stops.getState());
            telemetry.addLine();

            telemetry.addLine("LEFT CONTROLS (D-PAD)");
            telemetry.addData("^", "STOW");
            telemetry.addData("<", "HORIZONTAL");
            telemetry.addData("v", "PROTECT");

            telemetry.addLine();

            telemetry.addLine("RIGHT CONTROLS (BUTTONS)");
            telemetry.addData("Y", "STOW");
            telemetry.addData("B", "HORIZONTAL");
            telemetry.addData("A", "PROTECT");

            telemetry.update();
            idle();
        }
    }

}
