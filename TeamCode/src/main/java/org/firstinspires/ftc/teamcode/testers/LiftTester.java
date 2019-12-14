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

@TeleOp(name = "TestOp: Lift Tester")
public class LiftTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorA = hardwareMap.get(DcMotorEx.class, UniversalConstants.liftMotorA);
        motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotorEx motorB = hardwareMap.get(DcMotorEx.class, UniversalConstants.liftMotorB);
        motorB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Gamepad g = new Gamepad(this);
        PIDFCoefficients pidf = new PIDFCoefficients(5, 0.1, 0, 30);

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            motorA.setPower(Range.scale(g.lift,-1,1,-0.75,0.75));
            motorB.setPower(Range.scale(g.lift,-1,1,-0.75,0.75));
            telemetry.addData("Encoder A", motorA.getCurrentPosition());
            telemetry.addData("Encoder B", motorB.getCurrentPosition());
            telemetry.addData("Power A", motorA.getPower());
            telemetry.addData("Power B", motorB.getPower());
            telemetry.addData("P", motorB.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p);
            telemetry.addData("I", motorB.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i);
            telemetry.addData("D", motorB.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d);
            telemetry.addData("F", motorB.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
            g.update();
            telemetry.update();
        }
    }
}
