package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Lift;

@TeleOp(name = "TestOp: Fast Lift")
public class FastLift extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap,null);
//        Clamp clamp = new Clamp(this);
        Gamepad gamepad = new Gamepad(this);

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            lift.updateTargetLevelUsingGamepad(gamepad);
//            clamp.requestState(Clamp.ClampState.CLAMP);
            lift.update();
//            clamp.update();
            gamepad.update();
            telemetry.addData("Extend", gamepad.extendLift);
            telemetry.addLine(lift.getStatus());
            telemetry.update();
            idle();
        }
    }
}
