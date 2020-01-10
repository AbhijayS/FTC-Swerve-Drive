package org.firstinspires.ftc.teamcode.modules.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.*;

@TeleOp(group = "TestOp", name = "DEBUGGING: Servo Calibration")
public class Calibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Debugger robotDebugger = new Debugger(Util.getContext(), this, (ArrayList<String>) Debugger.Marker.getDebuggingMarkers());
        robotDebugger.initialize("servo_calib");
        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        SwerveModule[] modules = {swerveDrive.module0, swerveDrive.module1, swerveDrive.module3, swerveDrive.module2};

        double increment = 0.1;
        double angle = servoDefaultAngle;
        int frontBack = 0;
        int leftRight = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up)
                frontBack = 0;
            else if (gamepad1.dpad_down)
                frontBack = 2;

            if (gamepad1.dpad_left)
                leftRight = 0;
            else if (gamepad1.dpad_right) {
                leftRight = 1;
            }

            if (gamepad1.right_bumper)
                angle += increment;
            else if (gamepad1.left_bumper)
                angle -= increment;
            else if (gamepad1.left_stick_button)
                angle = servoDefaultAngle+90;
            else if (gamepad1.right_stick_button)
                angle = servoDefaultAngle-90;
            else if (gamepad1.y)
                angle = servoDefaultAngle;

            modules[frontBack+leftRight].swivel(angle);

            telemetry.addLine(String.format("Offset from home: %s deg", angle-servoDefaultAngle));
            telemetry.update();
        }

//        robotDebugger.stopLogging();
    }
}
