package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;

@TeleOp(name = "TestOp: Motor Swatter")
public class MotorSwatter extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g = new Gamepad(this);
        DcMotor left = hardwareMap.get(DcMotor.class, "JSLA1");
        DcMotor right = hardwareMap.get(DcMotor.class, "JSRA1");
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setTargetPosition(0);
        right.setTargetPosition(0);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int low = 120;

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            left.setTargetPosition(low);
            right.setTargetPosition(low);
            left.setPower(0.75);
            right.setPower(0.75);
            telemetry.addData("Right", right.getCurrentPosition());
            g.update();
            telemetry.update();
        }
    }
}
