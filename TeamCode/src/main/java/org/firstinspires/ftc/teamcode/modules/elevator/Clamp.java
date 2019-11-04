package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.sensory.Potentiometer;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;

// TODO: Implement potentiometer for clamp
public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private CRServo servo;
    private double maxPower = 1;
    private ClampState clampState;
    private Potentiometer potentiometer;
    public enum ClampState {
        CLAMP(1),
        STOW(-1),
        COAST(0);

        public final int POWER;

        ClampState(int power) {
            this.POWER = power;
        }
    }
    private int enable; // -1 = unclamp, 1 = clamp, 0 = no movement


    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.clampState = ClampState.CLAMP;
        potentiometer = new Potentiometer("P", linearOpMode);
    }

    // TODO: Add restrictions if necessary
    public void requestState(ClampState newState) {
        this.clampState = newState;
        update();
        enable = 1;
    }

    public void updateByGamepad(Gamepad g) {
        //64-100
        if (g.clamp) {
            this.clampState = ClampState.CLAMP;
        } else if (g.stow){
            this.clampState = ClampState.STOW;
        } else {
            this.clampState = ClampState.COAST;
        }
        if(potentiometer.returnAngle() <= 64 && this.clampState.equals(ClampState.CLAMP)){
            enable = 0;
        }else if(potentiometer.returnAngle() >= 99 && this.clampState.equals(ClampState.CLAMP)){
            enable = 0;
        }
        update();
    }

    public void clamp() {
        this.enable = 1;
        servo.setPower(enable * maxPower);
    }

    public void stow() {
        this.enable = -1;
        servo.setPower(enable * maxPower);
    }

    public void update() {
        servo.setPower(maxPower * clampState.POWER);
    }

    public String getStatus() {
        return clampState.name();
    }
}
