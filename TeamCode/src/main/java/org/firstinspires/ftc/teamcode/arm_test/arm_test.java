package org.firstinspires.ftc.teamcode.arm_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="collins_drivetrain")
class arm_test extends LinearOpMode {
    public void runOpMode() {
//        catapultDrivetrain drivetrain = new catapultDrivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        Servo arm_servo = hardwareMap.servo.get("s1");
        Servo grabber_servo = hardwareMap.servo.get("s2");
        double arm_position = 1;
        double grabber_position = 0.5;
        waitForStart();
        while (opModeIsActive()){
            arm_servo.setPosition(arm_position);
            if (gamepad1.x){
                if (arm_position==0.16) {
                    arm_position = 0;
                }
                if (arm_position == 1) {
                    arm_position = 0.16;
                }
            }

            if (gamepad1.y) {
                if (grabber_position == 0.5) {
                    grabber_position = 0;
                }
                if (grabber_position == 0) {
                    grabber_position = 0.5;
                }
            }
        }
    }
}