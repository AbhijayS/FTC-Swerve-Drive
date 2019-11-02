package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;

public class JewelSwatter {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private JewelSwatterArm left, right;

    public JewelSwatter(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        right = new JewelSwatterArm(l, UniversalConstants.jewelSwatterRightAxis1, UniversalConstants.jewelSwatterRightAxis2, Servo.Direction.FORWARD);
        left = new JewelSwatterArm(l, UniversalConstants.jewelSwatterLeftAxis1, UniversalConstants.jewelSwatterLeftAxis2, Servo.Direction.REVERSE);

    }

    public void stowLeft() {
        left.stow();
    }

    public void clampLeft() {
        left.clamp();
    }

    public void preclampLeft() {
        left.preclamp();
    }

    public void stowRight() {
        right.stow();
    }

    public void clampRight() {
        right.clamp();
    }

    public void preclampRight() {
        right.preclamp();
    }

    public void stowAll(){
        right.stow();
        left.stow();
    }

    public void travellingLeftLow() {
        left.travellingLow();
    }
    public void travellingLeftHigh() {
        left.travellingHigh();
    }

    public void travellingRightHigh() {
        right.travellingHigh();
    }
    public void travellingRightLow() {
        right.travellingLow();
    }
}

