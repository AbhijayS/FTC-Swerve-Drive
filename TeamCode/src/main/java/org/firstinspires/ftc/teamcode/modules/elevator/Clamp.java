package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Direction;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;

public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private CRServo servo;
    private double maxPower = 1;
    private int enable; // -1 = unclamp, 1 = clamp, 0 = no movement

    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        enable = 1;
    }

    public void updateByGamepad(Gamepad g) {
        if (g.clamp) {
            this.enable = 1;
        } else if (g.unclamp){
            this.enable = -1;
        } else {
            this.enable = 0;
        }

        servo.setPower(enable * maxPower);
    }

    public void clamp() {
        this.enable = 1;
        servo.setPower(enable * maxPower);
    }

    public void stow() {
        this.enable = -1;
        servo.setPower(enable * maxPower);
    }

    public void coast() {
        this.enable = 0;
        servo.setPower(enable * maxPower);
    }

    public String getStatus() {
        return enable == 1 ? "Clamp" : "Unclamp";
    }
}
