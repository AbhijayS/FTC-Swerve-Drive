package org.firstinspires.ftc.teamcode.modules.jewelSwatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="jewelSwatter")

public class jewelSwatter {

    Servo top_left;
    Servo bottom_left;
    double bottom_left_start = 0;
    double top_left_start = 0;
    double goto_top_left = top_left_start;
    double goto_bottom_left = bottom_left_start;

    public class extends jewelSwatter extends LinearOpMode {

        this.bottom_left = linearOpMode.hardwareMap.get(Servo.class, bottom_left);
        this.top_left = linearOpMode.hardwareMap.get(Servo.class, top_left);

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
            }
            if (gamepad2.y) {
                if (goto_top_left == 0) {
                    goto_top_left = 0.343;
                }
                if (goto_top_left == 0.343) {
                    goto_top_left = 0;
                }
            }
        }
    }

}