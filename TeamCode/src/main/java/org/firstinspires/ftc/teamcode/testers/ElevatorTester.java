package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Elevator;

@TeleOp(name = "Elevator Tester")
public class ElevatorTester extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException{
        Elevator elevator = new Elevator(this);

        Gamepad gamepad = new Gamepad(this);
        waitForStart();
        while (opModeIsActive()){
            elevator.updateByGamepad(gamepad);

        }
    }

}
