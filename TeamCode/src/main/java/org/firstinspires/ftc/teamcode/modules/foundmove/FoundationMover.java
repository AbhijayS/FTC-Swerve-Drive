package org.firstinspires.ftc.teamcode.modules.foundmove;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.foundMoveLeft;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.foundMoveRight;

public class FoundationMover {
    Servo left;
    Servo right;
    LinearOpMode linearOpMode;
    FoundationMover(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.left = linearOpMode.hardwareMap.get(Servo.class, foundMoveLeft);
        this.right = linearOpMode.hardwareMap.get(Servo.class, foundMoveRight);
    }
    public void clamp() {
        this.left.setPosition(0.7);
        this.right.setPosition(0.7);
        telemetry.addline("clamped");
        telemetry.update();
    }
    public void unclamp() {
        this.left.setPosition(0);
        this.right.setPosition(0);
        telemetry.addLine("unclamped");
        telemetry.update();
    }
}
