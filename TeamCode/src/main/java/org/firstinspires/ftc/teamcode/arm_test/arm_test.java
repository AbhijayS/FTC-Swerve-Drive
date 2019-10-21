package org.firstinspires.ftc.teamcode.arm_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="arm_test")
public class arm_test extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Servo arm_servo = hardwareMap.servo.get("s1");
        Servo grabber_servo = hardwareMap.servo.get("s2");
        double arm_position = 0.751;
        double grabber_position = 0.5;
        waitForStart();
        while (opModeIsActive()){
            arm_servo.setPosition(arm_position);
            if (gamepad1.x){
                if (arm_position==0.751) {
                    arm_position = 0.0455;
                } else if (arm_position == 1) {
                    arm_position = 0.16;
                } else {
                    arm_position = 1;
                }
                //waits until you release x
                while (gamepad1.x) {
                    wait(1);
                }  
            }

            if (gamepad1.y) {
                if (grabber_position == 0.343) {
                    grabber_position = 0;
                }
                if (grabber_position == 0) {
                    grabber_position = 0.343;
                }
                while (gamepad1.y) {
                    wait(1);
                }
            }
        }
    }
}