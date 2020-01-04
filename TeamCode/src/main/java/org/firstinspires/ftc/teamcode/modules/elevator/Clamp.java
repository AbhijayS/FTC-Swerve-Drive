package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;

// TODO: Implement potentiometer for clamp
public class Clamp {
    private CRServo servo;
    private ClampState clampState;
    private double position;
    private AnalogInput potentiometer;
    private double kP = 1.2;
    private double cerr = 0;
    private double kI = 0.02;

    public enum ClampState {
        CLAMP(6),
        STOW(2.48),
        PARTIAL(3),
        APPROACH(3.3),
        COAST(0);

        public final double position;

        ClampState(double position) {
            this.position= position;
        }
    }

    public Clamp(HardwareMap hardwareMap) {
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.clampState = ClampState.PARTIAL;
        potentiometer = hardwareMap.analogInput.get("P");
    }

    // TODO: Add restrictions if necessary
    public void requestState(ClampState newState) {
        this.clampState = newState;
    }

    public void updateByGamepad(Gamepad g) {
        //64-100
        if (g.clamp) {
            requestState(ClampState.CLAMP);
        }
        if (g.approach) {
            requestState(ClampState.APPROACH);
        }
        if (g.partial) {
            requestState(ClampState.PARTIAL);
        }

        update();
    }

    private double turnPID() {
        double error = clampState.position - position;
        cerr += error;
        if (Math.abs(error)<= 0.001)
            cerr = 0;
        return error*kP + cerr*kI;
    }

    public double update() {
        position = potentiometer.getVoltage();
        double power;
        if (clampState == ClampState.COAST) {
            power = 0;
            cerr = 0;
        } else if (clampState == ClampState.CLAMP) {
            power = 1;
            cerr = -1;
        } else
            power = turnPID();
        servo.setPower(power);
        return power;
    }

    public String getStatus() {
        return "Clamp: " + clampState.name() + " " + position + " " + cerr;
    }
}
