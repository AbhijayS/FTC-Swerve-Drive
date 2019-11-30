package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

@TeleOp(name = "Broken encoder")
public class LoopTimeTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this, null);

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData(swerveDrive.module2.toString(), swerveDrive.module2.driveMotor.getCurrentPosition());
            telemetry.addData(swerveDrive.module3.toString(), swerveDrive.module3.driveMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
