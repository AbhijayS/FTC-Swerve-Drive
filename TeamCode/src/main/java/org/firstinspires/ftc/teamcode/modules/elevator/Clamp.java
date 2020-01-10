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
    private double kP = 2;

    public enum ClampState {
        CLAMP(6),
        STOW(0.78),
        PARTIAL(0.937),
        APPROACH(1),
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
        if (g.clampStowed) {
            requestState(ClampState.STOW);
        }

        update();
    }

    private double turnPID() {
        double error = clampState.position - position;
        return error*kP;
    }

    public double update() {
        position = potentiometer.getVoltage();
        double power;
        if (clampState == ClampState.COAST) {
            power = 0;
        } else if (clampState == ClampState.CLAMP) {
            power = 1;
        } else
            power = turnPID();
        servo.setPower(power);
        return power;
    }

    public String getStatus() {
        return "Clamp: " + clampState.name() + " " + position;
    }
}
