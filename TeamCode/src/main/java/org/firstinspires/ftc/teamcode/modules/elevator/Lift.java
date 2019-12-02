package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;

public class Lift {
    private Debugger debugger;

    private DcMotor motorA, motorB;

    // TODO: Test if step over is accurate
    private final int STEP_OVER = 5; // inches
    private final int MIN_LEVEL = 0; // number of levels
    private final int MAX_LEVEL = 6; // number of levels

    // TODO: Determine level 1 height imperically
    private final int LEVEL_1_HEIGHT = 2; // inches

    private int targetHeight; // number of levels

    public Lift(HardwareMap hardwareMap, Debugger debugger) {
        this.debugger = debugger;
        this.motorA = hardwareMap.dcMotor.get(UniversalConstants.liftMotorA);
        this.motorA = hardwareMap.dcMotor.get(UniversalConstants.liftMotorB);
        this.targetHeight = 0;
    }

    // TODO: finish this function
    // make sure target height is within bounds
    // implement a saved position that the lift can go back to
    public int setTargetHeight(int height) {
        if (height >= MIN_LEVEL && height <= MAX_LEVEL) {
            targetHeight = height:
            return 0;
        }
        return -1;
    }
    
    public void update() {
        double targetPosition;
        double targetEncoder;
        if (targetHeight == 0) {
            targetPosition = 0;
            targetEncoder = 0;
        } else {
            targetPosition = (targetHeight-1)*STEP_OVER + LEVEL_1_HEIGHT;
            targetEncoder = convertToTicks(targetPosition);
        }
        motorA.setTargetPosition(targetEncoder);
        motorB.setTargetPosition(targetEncoder);
    }

    // TODO: Write an update function
    // updates using gamepad to increment (or decrement) the level height
    // probably write a simple increment height function with bounds
    // need to add some kind of incrementor to gamepad class
    // goes to position using PID

    // TODO: Implement bulk reading
    
    public double convertToTicks(double inches) {
        double tickValue = 0;
        double linear_conversion = inches / 4.0;
        tickValue = linear_conversion * (1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0;
        return tickValue;
    }
}
