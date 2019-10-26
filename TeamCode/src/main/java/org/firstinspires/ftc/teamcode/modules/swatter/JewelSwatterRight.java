package org.firstinspires.ftc.teamcode.modules.swatter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.jewelSwatterRightAxis1;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.jewelSwatterRightAxis2;

public class JewelSwatterRight {
    private Servo axis1;
    private Servo axis2;
    private LinearOpMode linearOpMode;

    public JewelSwatterRight(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        axis1 = linearOpMode.hardwareMap.get(Servo.class, jewelSwatterRightAxis1);
        axis2 = linearOpMode.hardwareMap.get(Servo.class, jewelSwatterRightAxis2);
    }

    public void clamp() {
        axis1.setPosition(0.751);
        axis2.setPosition(0.5);
    }

    public void unclamp() {
        axis1.setPosition(0);
        axis2.setPosition(0);
    }
}
