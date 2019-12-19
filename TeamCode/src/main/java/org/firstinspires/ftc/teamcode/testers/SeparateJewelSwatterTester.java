package org.firstinspires.ftc.teamcode.testers;

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
        DcMotorEx axis1 = hardwareMap.get(DcMotorEx.class, "JSRA1");
        Servo axis2 = hardwareMap.servo.get("JSRA2");
        axis1.setDirection(DcMotorSimple.Direction.REVERSE);
        axis2.setDirection(Servo.Direction.FORWARD);
        axis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axis1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            axis1.setPower(0.5);
            axis2.setPosition(0);
        }
    }
}
