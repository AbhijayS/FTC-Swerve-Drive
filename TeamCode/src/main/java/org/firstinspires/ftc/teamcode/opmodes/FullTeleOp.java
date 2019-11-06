package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Elevator;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

@TeleOp (name = "Opmode: TeleOp")
public class FullTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        Elevator elevator = new Elevator(this);
        //Clamp clamp = new Clamp(this);
        Gamepad gamepad = new Gamepad(this);
        double liftPower = 0;
        gamepad.update();



        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            //clamp.updateByGamepad(gamepad);
            telemetry.addData("Clamp status", elevator.clamp.getStatus());

            elevator.updateByGamepad(gamepad, liftPower);

            swerveDrive.fod(gamepad);
//            swerveDrive.swerveKinematics.update();
            gamepad.update();
            liftPower = gamepad.lift;
            telemetry.update();
        }
    }
}
