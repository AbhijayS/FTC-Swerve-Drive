package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class VEX393Servo {
    public CRServo servo;
    private double servoPower, maxSpeed, minSpeed;
    private DcMotorSimple.Direction servoDirection;

    public VEX393Servo(CRServo s, double max, double min) {
        servoDirection = s.getDirection();
        servoPower = s.getPower();
        maxSpeed = Math.abs(max);
        minSpeed = Math.abs(min);
        servo = s;
    }

    public void setPower(double power) {
        servoPower = Range.clip(power, -1, 1);
        if (power > 0) {
            servo.setPower(power * maxSpeed);
        } else {
            servo.setPower(power * minSpeed);
        }
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        if (servoDirection != direction) {
            servoDirection = direction;
            maxSpeed *= -1;
            minSpeed *= -1;
        }
    }

    public double getPower() {
        return servoPower;
    }

}
