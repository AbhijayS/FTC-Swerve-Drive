package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

@Deprecated
public class Lift {
    private Debugger debugger;

    private DcMotor motorA, motorB;

    // TODO: Test if step over is accurate
    private final int STEP_OVER = 5; // inches

    // Level 0 = Stowed
    // Level 1 = 1st Stone
    // Level N = Nth Stone
    private final int MIN_LEVEL = 0; // number of levels
    private final int MAX_LEVEL = 6; // number of levels

    // TODO: Determine level 1 height imperically
    private final int LEVEL_1_HEIGHT = 2; // inches

    private int targetLevel; // number of levels
    public enum State {
        STOW,
        EXTEND
    }

    public Lift(HardwareMap hardwareMap, Debugger debugger) {
        this.debugger = debugger;
        this.motorA = hardwareMap.dcMotor.get(UniversalConstants.liftMotorA);
        this.motorA = hardwareMap.dcMotor.get(UniversalConstants.liftMotorB);
        this.targetLevel = 0;
    }

    // TODO: finish this function
    // make sure target height is within bounds
    // implement a saved position that the lift can go back to
    public int setTargetLevel(int level) {
        if (level >= MIN_LEVEL && level <= MAX_LEVEL) {
            targetLevel = level;
            return 0;
        }
        return -1;
    }
    
    public void update() {
        int targetEncoder;
        if (targetLevel == 0) {
            targetEncoder = 0;
        } else {
            int targetHeight;
            targetHeight = (targetLevel-1) * STEP_OVER + LEVEL_1_HEIGHT;
            targetEncoder = convertToTicks(targetHeight);
        }
        motorA.setTargetPosition(targetEncoder);
        motorB.setTargetPosition(targetEncoder);
    }

    public void updateTargetLevelUsingGamepad(Gamepad gamepad) {
//        if (gamepad.incrementLevel)
//            incrementLevel();
//        else if (gamepad.decrementLevel)
//            decrementLevel();
//        else if (gamepad.stowLift)
//            targetLevel = MIN_LEVEL;
//        else if (gamepad.extendLift)
//            setTargetLevel(savedLevel);

    }

    public void incrementLevel() {
        setTargetLevel(targetLevel + 1);
    }

    public void decrementLevel() {
        setTargetLevel(targetLevel - 1);
    }


    // TODO: Write an update function
    // updates using gamepad to increment (or decrement) the level height
    // probably write a simple increment height function with bounds
    // need to add some kind of incrementor to gamepad class
    // goes to position using PID

    // TODO: Implement bulk reading
    private int convertToTicks(double inches) {
        double tickValue = 0;
        double linear_conversion = inches / 4.0;
        tickValue = linear_conversion * (1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0;
        return (int) Math.round(tickValue);
    }
}
