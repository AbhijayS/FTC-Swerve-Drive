package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

@TeleOp(name = "TestOp: Drive PID")
public class DrivePID extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        swerveDrive.swivel(90);
        swerveDrive.enablePID();

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        swerveDrive.swivel(0);
        Thread.sleep(500);
        swerveDrive.movePID(24, 1);

        while (opModeIsActive()) {
            telemetry.addLine("Moving to 10 inches");
            telemetry.update();
        }
    }
}
