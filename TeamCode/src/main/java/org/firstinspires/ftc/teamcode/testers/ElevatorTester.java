package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Elevator;

@TeleOp(name = "TestOp: Elevator")
public class ElevatorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Elevator elevator = new Elevator(this);
        //Clamp clamp = new Clamp(this);
        Gamepad gamepad = new Gamepad(this);
        double liftPower = 0;
        waitForStart();
        while (opModeIsActive()) {
            elevator.updateByGamepad(gamepad, liftPower);
            //telemetry.addData("Clamp status", clamp.getStatus());
            /*double power = Range.scale(gamepad.lift, -1, 1, -0.25, 0.25);
            liftOne.setPower(power);
            liftTwo.setPower(power);*/
            gamepad.update();
            liftPower = gamepad.lift;
            //telemetry.update();
        }
    }

}
