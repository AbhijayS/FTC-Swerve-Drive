package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.HALF_PI;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.PI;

public class Lift {
    // TODO: Test if step over is accurate
    private final int STEP_OVER = 4; // inches
    // TODO: Determine level 1 height imperically
    private final int LEVEL_1_HEIGHT = 2; // inches
    // TODO: Determine feedforward power imperically
    private final double kf = 0;
    private PIDFCoefficients pidf;

    // Level 1 = 1st Stone
    // Level N = Nth Stone
    private final int MIN_LEVEL = 1; // number of levels
    private final int MAX_LEVEL = 6; // number of levels

    private Debugger debugger;
    private DcMotorEx motorA, motorB;
    private int targetLevel; // saved level used for State.EXTEND
    private State state; // lift state

    public enum State {
        STOW, // always goes to level 0 to fit under the alliance specific bridge
        EXTEND // extends to a level between MIN_LEVEL and MAX_LEVEL, inclusive
    }

    public Lift(HardwareMap hardwareMap, Debugger debugger) {
        this.debugger = debugger;
        this.motorA = hardwareMap.get(DcMotorEx.class, UniversalConstants.liftMotorA);
        this.motorB = hardwareMap.get(DcMotorEx.class, UniversalConstants.liftMotorB);
        this.motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorB.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorA.setTargetPosition(0);
        this.motorB.setTargetPosition(0);
        this.motorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorA.setVelocityPIDFCoefficients(2,0.25,0,30);
        this.motorB.setVelocityPIDFCoefficients(2,0.25,0,30);
        this.targetLevel = MIN_LEVEL;
        this.state = State.STOW;
    }

    public int setTargetLevel(int level) {
        if (level >= MIN_LEVEL && level <= MAX_LEVEL) {
            targetLevel = level;
            return 0;
        }
        return -1;
    }

    public int requestState(State newState) {
        this.state = newState;
        return 0;
    }
    
    public void update() {
        int targetEncoder;

        switch (state) {
            case EXTEND: {
                int targetHeight = (targetLevel-1) * STEP_OVER + LEVEL_1_HEIGHT - 1;
                targetEncoder = convertToTicks(targetHeight);
                break;
            }

            default: {
                targetEncoder = 0;
                break;
            }
        }

        motorA.setTargetPosition(targetEncoder);
        motorB.setTargetPosition(targetEncoder);
        motorA.setPower(0.8);
        motorB.setPower(0.8);
    }

    public int updateTargetLevelUsingGamepad(Gamepad gamepad) {
        int status = 0;

        if (gamepad.incrementLift)
            status = incrementLevel();
        if (gamepad.decrementLift)
            status = decrementLevel();

        if (gamepad.stowLift) {
            status = requestState(State.STOW);
            update();
        }
        if (gamepad.extendLift) {
            status = requestState(State.EXTEND);
            update();
        }
        return status;
    }

    public int incrementLevel() {
        return setTargetLevel(targetLevel + 1);
    }

    public int decrementLevel() {
        return setTargetLevel(targetLevel - 1);
    }

    // TODO: Implement bulk reading

    // TODO: Test convertToTicks method
    private int convertToTicks(double inches) {
//        double tickValue = 0;
//        double linear_conversion = inches / 4.0;
//        tickValue = linear_conversion * (1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0;
//        return (int) Math.round(tickValue);
        return (int) Math.round(inches*1.63*560.0/(4.0*2.0*PI*1.15));
    }

    // quick status of the lift
    public String getStatus() {
        String status = "Lift: " + state.name();
        if (state==State.EXTEND)
            status += " " + targetLevel;
        return status;
    }

    // more detailed description of the lift
    public void log() {
        debugger.addData("Lift State", state.name());
        debugger.addData("Lift Level", targetLevel);
        int encoderAvg = (motorA.getCurrentPosition() + motorB.getCurrentPosition())/2;
        debugger.addData("Lift Encoder", encoderAvg);
    }

    @Override
    public String toString() {
        return getStatus();
    }
}
