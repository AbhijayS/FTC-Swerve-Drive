package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;

import java.util.ArrayList;

@TeleOp(name = "TestOp: Swerve")
public class TurnDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) UniversalConstants.Debugging.getDebuggingMarkers());
        robotDebugger.initialize("turn drive");
        Gamepad gamepad = new Gamepad(this);
        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            swerveDrive.swerveKinematics.update();
            swerveDrive.fod(gamepad);
            gamepad.update();
            robotDebugger.log();
        }
        robotDebugger.stopLogging();
    }
}
