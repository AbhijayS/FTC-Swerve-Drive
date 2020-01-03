package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Lift;

@TeleOp(name = "Clamp + Lift")
public class ElevatorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Clamp clamp = new Clamp(hardwareMap);
        Lift lift = new Lift(hardwareMap,null);
        Gamepad g = new Gamepad(this);
        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            clamp.updateByGamepad(g);
            lift.updateTargetLevelUsingGamepad(g);
            lift.update();
            telemetry.addLine(lift.getStatus());
            telemetry.addData("Clamp Power", clamp.update());
            g.update();
            telemetry.update();
        }
    }
}
