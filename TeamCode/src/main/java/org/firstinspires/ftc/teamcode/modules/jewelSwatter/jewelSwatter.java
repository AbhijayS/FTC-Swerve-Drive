package org.firstinspires.ftc.teamcode.modules.jewelSwatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="jewelSwatter")

public class jewelSwatter extends LinearOpMode{

    Servo top_left = hardwareMap.servo.get("s1");
    Servo bottom_left = hardwareMap.servo.get("s2");

    double bottom_left_start = 0;
    double top_left_start = 0;
    double goto_top_left = top_left_start;
    double goto_bottom_left = bottom_left_start;

    public void runOpMode() throws InterruptedException{
        waitForStart();

        bottom_left.setPosition(bottom_left_start);
        top_left.setPosition(top_left_start);


        while (opModeIsActive()) {

            top_left.setPosition(goto_top_left);
            bottom_left.setPosition(goto_bottom_left);

            if (gamepad2.x) {
                if (goto_bottom_left == 0.751) {
                    goto_bottom_left = 0;
                } else if (goto_bottom_left == 0) {
                    goto_bottom_left = 0.751;
                }
                while (gamepad2.x) {

                    //how about we stop commenting
                }
            }
            if (gamepad2.y) {
                if (goto_top_left == 0) {
                    goto_top_left = 0.343;
                } else if (goto_top_left == 0.343) {
                    goto_top_left = 0;
                }
                while (gamepad2.y) {
                    //how about we stop commenting

                }
            }
        }
    }
}