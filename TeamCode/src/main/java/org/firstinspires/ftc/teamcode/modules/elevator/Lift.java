package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;

// TODO: Implement bulk reading
public class Lift {
    private Debugger debugger;

    private DcMotor motorA, motorB;

    private final int STEP_OVER = 5; // inches
    private final int MIN_HEIGHT = 0; // inches
    private final int MAX_HEIGHT = 40; // inches

    public Lift(HardwareMap hardwareMap, Debugger debugger) {
        this.debugger = debugger;
        this.motorA = hardwareMap.dcMotor.get(UniversalConstants.liftMotorA);
        this.motorA = hardwareMap.dcMotor.get(UniversalConstants.liftMotorB);
    }
}
