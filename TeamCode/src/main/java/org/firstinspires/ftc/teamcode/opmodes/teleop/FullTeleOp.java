package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Elevator;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

import java.util.ArrayList;

@TeleOp (name = "Opmode: TeleOp")
public class FullTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("TeleOp");
        SwerveDrive swerveDrive = new SwerveDrive(this, robotDebugger);
        Elevator elevator = new Elevator(this);
        Clamp clamp = new Clamp(this);
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
