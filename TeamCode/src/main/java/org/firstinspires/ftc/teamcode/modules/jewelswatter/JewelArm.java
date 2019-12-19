package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.JewelSwatterConfig;

class JewelArm {
    private DcMotorEx axis1;
    private Servo axis2;
    private JewelArmState state;

    private DcMotor.RunMode axis1Mode;
    private double axis1Power;
    private int axis1Position, axis2Position;

    JewelArm(HardwareMap hardwareMap, JewelSwatterConfig config) {
        this.axis1 = hardwareMap.get(DcMotorEx.class, config.nameAxis1);
        this.axis2 = hardwareMap.servo.get(config.nameAxis2);

        // happen only once
        this.axis1.setDirection(config.directionAxis1);
        this.axis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.axis2.setDirection(config.directionAxis2);

        this.state = JewelArmState.STOW;
        this.axis1Mode = DcMotor.RunMode.RUN_TO_POSITION;
        this.axis1Power = 0.75;
        this.axis1Position = state.axis1;
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
        DcMotor.RunMode a1Mode;
        double a1Power;
        int a1Position;
        int a2Position;

        /*
        update new values according to the current state
         */
        a1Position = this.state.axis1;
        a2Position = this.state.axis2;

        if (state == JewelArmState.DOCK || state == JewelArmState.POSSESS) {
            a1Mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            a1Power = 0.5;
        } else {
            a1Mode = DcMotor.RunMode.RUN_TO_POSITION;
            a1Power = 0.75;
        }

        /*
        redundancy filter to limit hardware communication
        TODO: Move all hardware communications to a separate thread
         */
        if (a1Position != axis1Position) {
            this.axis1.setTargetPosition(a1Position);
            this.axis1Position = a1Position;
        }

        if (a1Mode != axis1Mode) {
            this.axis1.setMode(a1Mode);
            this.axis1Mode = a1Mode;
        }

        if (a1Power != axis1Power) {
            this.axis1.setPower(a1Power);
            this.axis1Power = a1Power;
        }

        if (a2Position != axis2Position) {
            this.axis2.setPosition(a2Position);
            this.axis2Position = a2Position;
        }
    }

    public enum JewelArmState {
        STOW(0, 0), DOCK(160, 1), POSSESS(150, 0), RIDE_LOW(130, 0), RIDE_LOW_OPEN(130,1), RIDE_HIGH(80, 0), RIDE_HIGH_OPEN(80,1);
        public int axis1, axis2;

        JewelArmState(int axis1, int axis2) {
            this.axis1 = axis1;
            this.axis2 = axis2;
        }
    }
}