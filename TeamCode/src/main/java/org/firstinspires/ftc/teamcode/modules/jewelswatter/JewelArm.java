package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.JewelSwatterConfig;

class JewelArm {
    private DcMotorEx axis1;
    private Servo axis2;
    private JewelArmState state;
    private JewelSwatterConfig config;

    JewelArm(HardwareMap hardwareMap, JewelSwatterConfig config) {
        this.axis1 = hardwareMap.get(DcMotorEx.class, config.nameAxis1);
        this.axis2 = hardwareMap.servo.get(config.nameAxis2);

        this.axis1.setDirection(config.directionAxis1);
        this.axis1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.axis2.setDirection(config.directionAxis2);

        this.state = JewelArmState.STOW;
        this.config = config;

        update();

        this.axis1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.axis1.setPower(1);
    }

    // TODO: add restricttions to states using switch statements
    public int requestState(JewelArmState newState) {
        this.state = newState;
        return 0;
    }

    public void update() {
        this.axis1.setTargetPosition(state.axis1);
        this.axis2.setPosition(state.axis2);
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