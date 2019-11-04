package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;

// TODO: Implement potentiometer for clamp
public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private CRServo servo;
    private double maxPower = 1;
    private ClampState clampState;

    public enum ClampState {
        CLAMP(1),
        STOW(-1),
        COAST(0);

        public final int POWER;

        ClampState(int power) {
            this.POWER = power;
        }
    }

    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.clampState = ClampState.CLAMP;
    }

    // TODO: Add restrictions if necessary
    public void requestState(ClampState newState) {
        this.clampState = newState;
        update();
    }

    public void updateByGamepad(Gamepad g) {
        if (g.clamp) {
            this.clampState = ClampState.CLAMP;
        } else if (g.stow){
            this.clampState = ClampState.STOW;
        } else {
            this.clampState = ClampState.COAST;
        }
        update();
    }

    public void update() {
        servo.setPower(maxPower * clampState.POWER);
    }

    public String getStatus() {
        return clampState.name();
    }
}
