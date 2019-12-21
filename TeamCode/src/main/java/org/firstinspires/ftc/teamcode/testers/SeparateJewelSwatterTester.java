package org.firstinspires.ftc.teamcode.testers;

import android.nfc.TagLostException;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DEBUGGING: Jewel Swatter")
public class SeparateJewelSwatterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx axis1 = hardwareMap.get(DcMotorEx.class, "JSLA1");
        Servo axis2 = hardwareMap.servo.get("JSLA2");
        axis1.setDirection(DcMotorSimple.Direction.FORWARD);
        axis2.setDirection(Servo.Direction.FORWARD);
        axis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axis1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        axis2.setPosition(1);
        double target = 144.0;
        double fMax = 0.15;
        double p = 1.0/144.0;
        double i = 0.00001;
        double d = 0.001;
        double cerr = 0;
        waitForStart();

        while (opModeIsActive()) {
            // UPDATE
            double pos = axis1.getCurrentPosition();
            double vel = axis1.getVelocity();
            double error = target-pos;
            double f = Math.cos(Math.toRadians(ticksToAngle(pos)))*fMax;
            double power = (error*p) + (cerr * i) + (-vel * d) + f;
            cerr += error;
            axis1.setPower(power);

            telemetry.addData("Encoder", pos);
            telemetry.addData("Error", error);
            telemetry.addData("Feedforward", f);
            telemetry.addData("Power", power);
            telemetry.update();
            idle();
        }
    }

    public double ticksToAngle(double ticks) {
        double ratio = 360.0/288.0;
        return ratio * ticks;
    }
}
