package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.JewelSwatterConfig;

public class JewelArm {
    private DcMotorEx axis1;
    private Servo axis2;
    private JewelArmState state;
    private ExpansionHubEx hubEx;
    private RevBulkData bulkData;
    private Debugger debugger;

    private double axis1Power;
    private int axis1Position, axis2Position;
    private DcMotor.RunMode axis1Mode;

    private double fStone = 0.15;
    private double p = 1.0/80.0;
    private double i = 0.00001;
    private double d = 0.001;
    private double cerr = 0;

    public JewelArm(HardwareMap hardwareMap, JewelSwatterConfig config, Debugger debugger) {
        this.axis1 = hardwareMap.get(DcMotorEx.class, config.nameAxis1);
        this.axis2 = hardwareMap.servo.get(config.nameAxis2);
        this.debugger = debugger;

        // happen only once
        this.axis2.setDirection(config.directionAxis2);
        this.axis1.setDirection(config.directionAxis1);
        this.axis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.axis1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize variables
        requestState(JewelArmState.STOW);
        this.axis2Position = state.axis2;
        update();
    }

    // TODO: add restricttions to states using switch statements
    public int requestState(JewelArmState newState) {
        this.state = newState;
        return 0;
    }

    public void update() {
        // new values
        double a1Power;
        int a1Position = state.axis1;
        int a2Position = state.axis2;
        DcMotor.RunMode a1Mode;

        if (state == JewelArmState.DOCK || state == JewelArmState.POSSESS) {
            a1Mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            a1Power = 0.8;
        } else {
            a1Mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            a1Power = PIDF();
        }
        this.axis2.setPosition(a2Position);

        /*
        redundancy filter to limit hardware communication
        TODO: Move all hardware communications to a separate thread
         */

//        if (a2Position != axis2Position) {
//            this.axis2.setPosition(a2Position);
//            this.axis2Position = a2Position;
//        }


        if (a1Mode != axis1Mode) {
            this.axis1.setMode(a1Mode);
            this.axis1Mode = a1Mode;
        }

        if (a1Power != axis1Power) {
            this.axis1.setPower(a1Power);
            this.axis1Power = a1Power;
        }

        if (a1Position != axis1Position) {
            cerr = 0;
            this.axis1Position = a1Position;
        }

    }

    private double PIDF() {
        double pos = axis1.getCurrentPosition();
        double vel = axis1.getVelocity();
        double error = state.axis1-pos;
        double f = Math.cos(Math.toRadians(ticksToAngle(pos)))*fStone;
        double power = (error*p) + (cerr * i) + (-vel * d) + f;
        cerr += error;
//        debugger.addData("Encoder", pos);
//        debugger.addData("Power", power);
//        debugger.addData("Feed forward", fDrive);
//        debugger.addData("Error", error);
//        debugger.addData("Cumulative", cerr);

        return power;
    }

    private double ticksToAngle(double ticks) {
        double ratio = 360.0/288.0;
        return ratio * ticks;
    }

    public enum JewelArmState {
        STOW(0, 0), DOCK(168, 1), POSSESS(168, 0), RIDE_LOW(130, 0), RIDE_LOW_OPEN(130,1), RIDE_HIGH(80, 0), RIDE_HIGH_OPEN(80,1);
        public int axis1, axis2;

        JewelArmState(int axis1, int axis2) {
            this.axis1 = axis1;
            this.axis2 = axis2;
        }
    }
}