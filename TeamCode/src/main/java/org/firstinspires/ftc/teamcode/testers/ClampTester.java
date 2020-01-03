package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;

@TeleOp(group = "TestOp", name = "Clamp")
public class ClampTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Clamp clamp = new Clamp(hardwareMap);
        Gamepad g = new Gamepad(this);
        waitForStart();

        while (opModeIsActive()) {
            clamp.updateByGamepad(g);
            g.update();
            telemetry.addData("Clamp", clamp.getStatus());
            telemetry.update();
        }
    }
}
