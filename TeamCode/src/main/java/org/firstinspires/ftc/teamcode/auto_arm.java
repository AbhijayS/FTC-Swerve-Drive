package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="collins_drivetrain")

public class auto_arm extends LinearOpMode {
    Servo   Servo;
    public void runOpMode() {
        Servo lift_servo = hardwareMap.get(Servo.class, "s1");
        Servo grab_servo = hardwareMap.get(Servo.class, "s2");
        waitForStart();
        // run until the end of the match (driver presses STOP)
        double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            // check to see if we need to move the servo.

            if(gamepad1.y) {
                // move to 0 degrees.
                lift_servo.setPosition(0);
            } else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees.
                lift_servo.setPosition(0.5);
            } else if (gamepad1.a) {
                // move to 180 degrees.
                lift_servo.setPosition(1);
            }
            telemetry.addData("Servo Position", lift_servo.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", ">>>");
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}