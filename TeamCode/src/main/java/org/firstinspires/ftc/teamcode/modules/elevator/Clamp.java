package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.sensory.Potentiometer;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;

public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private CRServo servo;
    private double maxPower = 1;
    private int enable; // -1 = unclamp, 1 = clamp, 0 = no movement
    private Potentiometer potentiometer;

    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        potentiometer = new Potentiometer("P", linearOpMode);
        enable = 1;

    }

    public void updateByGamepad(Gamepad g) {
        //64-100
        if (g.clamp) {
            this.enable = 1;
        } else if (g.unclamp){
            this.enable = -1;
        } else {
            this.enable = 0;
        }
        if(potentiometer.returnAngle() <= 64 && enable < 0){
            enable = 0;
        }else if(potentiometer.returnAngle() >= 99 && enable >0){
            enable = 0;
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

    public void noPower() {
        this.enable = 0;
        servo.setPower(enable * maxPower);
    }

    public String getStatus() {
        return enable == 1 ? "Clamp" : "Unclamp";
    }
}
