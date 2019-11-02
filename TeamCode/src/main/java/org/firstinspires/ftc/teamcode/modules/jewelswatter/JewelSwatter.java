package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.jewelSwatterLeftAxis1;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.jewelSwatterLeftAxis2;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.jewelSwatterRightAxis1;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.jewelSwatterRightAxis2;

public class JewelSwatter {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;

    private enum JewelArmConfig {
        LEFT(jewelSwatterLeftAxis1, jewelSwatterLeftAxis2, Servo.Direction.REVERSE, Servo.Direction.REVERSE, 0),
        RIGHT(jewelSwatterRightAxis1, jewelSwatterRightAxis2, Servo.Direction.FORWARD, Servo.Direction.FORWARD, 0);

        private Servo.Direction directionAxis1, directionAxis2;
        private double delta;
        public double STOWED_AXIS_1, STOWED_AXIS_2, DOCK_AXIS_1, DOCK_AXIS_2, LOW_AXIS_1, HIGH_AXIS_1;

        JewelArmConfig(String nameAxis1, String nameAxis2, Servo.Direction directionAxis1, Servo.Direction directionAxis2, double delta) {
            this.directionAxis1 = directionAxis1;
            this.directionAxis2 = directionAxis2;
            this.delta = delta;
        }
    }

    public JewelSwatter(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        right = new JewelSwatterArm(l, jewelSwatterRightAxis1, jewelSwatterRightAxis2, Servo.Direction.FORWARD);
        left = new JewelSwatterArm(l, jewelSwatterLeftAxis1, jewelSwatterLeftAxis2, Servo.Direction.REVERSE);

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

