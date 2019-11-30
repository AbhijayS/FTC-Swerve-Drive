package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.JewelSwatterConfig;

class JewelArm {
    private Servo axis1, axis2;
    private JewelArmState state;
    private JewelSwatterConfig config;

    JewelArm(HardwareMap hardwareMap, JewelSwatterConfig config) {
        this.axis1 = hardwareMap.servo.get(config.nameAxis1);
        this.axis2 = hardwareMap.servo.get(config.nameAxis2);

        this.axis1.setDirection(config.directionAxis1);
        this.axis2.setDirection(config.directionAxis2);

        this.state = JewelArmState.STOW;

        this.config = config;
    }

    // TODO: add restricttions to states using switch statements
    public int requestState(JewelArmState newState) {
        this.state = newState;
        return 0;
    }

    public void update() {
        this.axis1.setPosition(state.axis1 + config.delta);
        this.axis2.setPosition(state.axis2);
    }

    public enum JewelArmState {
        STOW(0, 0), DOCK(1, 1), POSSESS(1, 0), RIDE_LOW(0.7, 0), RIDE_LOW_OPEN(0.7,1), RIDE_HIGH(0.4, 0);
        public double axis1, axis2;

        JewelArmState(double axis1, double axis2) {
            this.axis1 = axis1;
            this.axis2 = axis2;
        }
    }
}