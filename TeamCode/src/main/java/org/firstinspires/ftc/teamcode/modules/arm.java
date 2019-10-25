package org.firstinspires.ftc.teamcode.arm_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class arm {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private Servo arm, claw;

    public arm(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        arm = hardwareMap.servo.get("a");
        claw = hardwareMap.servo.get("c");
        arm.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

    }
    public double deg2pos(double angle, double maxRange) {
        double conversion = angle/maxRange;
        return conversion;
    }
    public void setPositions(double arm, double claw) {
        this.arm.setPosition(deg2pos(arm, 180));
        this.claw.setPosition(deg2pos(claw, 90));
    }
    public void updateTelemetry() {
        telemetry.addData("arm pos: ", arm.getPosition());
        telemetry.addData("claw pos: ", claw.getPosition());
        telemetry.update();
    }

}
