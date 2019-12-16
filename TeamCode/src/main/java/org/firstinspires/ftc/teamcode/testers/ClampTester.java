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

@TeleOp(name = "TestOp: Clamp")
public class ClampTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Clamp clamp = new Clamp(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            clamp.requestState(Clamp.ClampState.STOW);
            clamp.update();
            telemetry.update();
        }
    }
}
