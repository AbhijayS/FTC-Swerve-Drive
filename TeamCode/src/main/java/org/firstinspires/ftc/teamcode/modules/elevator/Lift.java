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
        return -1;
    }

    // TODO: Write an update function
    // updates using gamepad to increment (or decrement) the level height
    // probably write a simple increment height function with bounds
    // need to add some kind of incrementor to gamepad class
    // goes to position using PID

    // TODO: Implement bulk reading
}
