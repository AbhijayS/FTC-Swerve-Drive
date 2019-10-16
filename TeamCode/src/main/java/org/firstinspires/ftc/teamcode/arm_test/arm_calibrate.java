package org.firstinspires.ftc.teamcode.arm_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="arm_test")

public class arm_calibrate extends LinearOpMode {
    Servo Servo;

    public void runOpMode() {
        Servo lift_servo = hardwareMap.get(Servo.class, "s1");
        Servo grab_servo = hardwareMap.get(Servo.class, "s2");
        double arm_position = 0.5;
        double grabber_position = 0.5;
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {

            if (gamepad1.y) {
                arm_position = arm_position + 0.0005;
            }
            if (gamepad1.x) {
                arm_position = arm_position - 0.0005;
            }
            if (gamepad1.a) {
                grabber_position = grabber_position + 0.0005;
            }
            if (gamepad1.b) {
                grabber_position = grabber_position - 0.0005;
            }
            lift_servo.setPosition(arm_position);
            lift_servo.setPosition(grabber_position);
            telemetry.addData("lift_Servo Position", arm_position);
            telemetry.addData("Grab_Servo position", grabber_position);
            telemetry.addLine("Status: Running");
            telemetry.update();

        }
    }
}