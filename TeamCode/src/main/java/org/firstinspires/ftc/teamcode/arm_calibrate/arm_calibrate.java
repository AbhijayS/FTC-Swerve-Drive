package org.firstinspires.ftc.teamcode.arm_calibrate;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="arm_calibrate")
public class arm_calibrate extends LinearOpMode {
    public void runOpMode() {
        Servo arm_servo = hardwareMap.servo.get("s1");
        Servo grabber_servo = hardwareMap.servo.get("s2");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("arm_servo position: ", arm_servo.getPosition());
            telemetry.addData("grabber_servo position: ", grabber_servo.getPosition());
            telemetry.update();
        }
    }
}