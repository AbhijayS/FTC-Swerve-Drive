package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.sensory.Potentiometer;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;

// TODO: Implement potentiometer for clamp
public class Clamp {
    private CRServo servo;
    private ClampState clampState;
    private double position;
    private AnalogInput potentiometer;
    private double kP = 0.8;

    public enum ClampState {
        CLAMP(3.34),
        STOW(2.48),
        PARTIAL(3),
        COAST(0);

        public final double position;

        ClampState(double power) {
            this.position= power;
        }
    }

    public Clamp(HardwareMap hardwareMap) {
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.clampState = ClampState.CLAMP;
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
        } else if (g.stow){
            requestState(ClampState.STOW);
        } else {
            requestState(ClampState.COAST);
        }
        update();
    }

    private double turnPID() {
        double error = clampState.position - position;
        return error*kP;
    }

    public void update() {
        position = potentiometer.getVoltage();
        servo.setPower(turnPID());
    }

    public String getStatus() {
        return clampState.name();
    }
}
