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
        double position = 0.5;
        double grabber_position = 0.5;
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {

            if (gamepad1.y) {
                position = position + 0.0005;
            }
            if (gamepad1.x) {
                position = position - 0.0005;
            }
            if (gamepad1.a) {
                position = position + 0.0005;
            }
            if (gamepad1.b) {
                grabber_position = grabber_position - 0.0005;
            }
            lift_servo.setPosition(position);
            telemetry.addData("lift_Servo Position", position);
//            telemetry.addData("Grab_Servo position", position);
            telemetry.addLine("Status: Running");
            telemetry.update();

        }
    }
}