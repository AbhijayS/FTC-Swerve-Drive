package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;

public class JewelSwatterArm {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private Servo arm, claw;
    private Servo.Direction direction;

    public JewelSwatterArm(LinearOpMode l, String arm, String claw, Servo.Direction direction) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        this.arm = hardwareMap.servo.get(arm);
        this.claw = hardwareMap.servo.get(claw);
        this.arm.setDirection(direction);
        this.claw.setDirection(direction);
    }

    public void stow() {
        arm.setPosition(0);
        claw.setPosition(0);
    }

    public void clamp() {
        claw.setPosition(0);
    }

    public void unclamp() {
        claw.setPosition(1);
    }

    public void preclamp() {
        arm.setPosition(1);
        claw.setPosition(1);
    }

    public void travellingHigh() {
        arm.setPosition(0.4);
        clamp();
    }

    public void travellingLow() {
        arm.setPosition(0.6);
        clamp();
    }
}
