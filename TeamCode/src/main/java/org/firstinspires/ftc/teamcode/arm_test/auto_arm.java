package org.firstinspires.ftc.teamcode.arm_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="arm_test")

public class auto_arm extends LinearOpMode {
    Servo Servo;

    public void runOpMode() {
        Servo lift_servo = hardwareMap.get(Servo.class, "s1");
        Servo grab_servo = hardwareMap.get(Servo.class, "s2");
        double position = 0;
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {

            if (gamepad1.y) {
                position = position + 0.0005;
//                grab_servo.setPosition(position);
            }
            if (gamepad1.x) {
                position = position - 0.0005;
//                grab_servo.setPosition(position-2);
//                if (position-2 < 5) {
//                    grab_servo.setPosition(position+2);
//                }
            }
            lift_servo.setPosition(position);
            telemetry.addData("lift_Servo Position", position);
//            telemetry.addData("Grab_Servo position", position);
            telemetry.addLine("Status: Running");
            telemetry.update();

        }
    }
}