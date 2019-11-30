package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.sensory.Potentiometer;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.clampServo;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kI;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.kP;

// TODO: Implement potentiometer for clamp
public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private CRServo servo;
    private ClampState clampState;
    private Potentiometer potentiometer;
    private int position;
    private double cerr;
    public enum ClampState {
        CLAMP(1, 274),
        STOW(-1, 200),
        APPROACH(1, 235),
        COAST(0, 274);

        public final int POWER;
        public final int POSITION;

        ClampState(int power, int position) {
            this.POWER = power;
            this.POSITION = position;
        }
    }

    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        potentiometer = new Potentiometer("P", linearOpMode);
        servo = hardwareMap.crservo.get(clampServo);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        this.clampState = ClampState.CLAMP;
    }

    private double turnPID(double targetAngle) {
//        if (lastTargetAngle != targetAngle)
//            cerr = 0;
        double err = targetAngle-position;
        double power;
        cerr += err;
        power = err * kP + cerr * kI;
//        debugger.addData("CERR", Double.toString(cerr));
        return power;
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

    public void update() {
        position = potentiometer.returnAngle();
//        if (clampState != ClampState.COAST)
//            servo.setPower(turnPID(clampState.POSITION));
        servo.setPower(clampState.POWER);
        linearOpMode.telemetry.addData("Angle", potentiometer.returnAngle());
    }

    public String getState() {
        return clampState.name();
    }
}
