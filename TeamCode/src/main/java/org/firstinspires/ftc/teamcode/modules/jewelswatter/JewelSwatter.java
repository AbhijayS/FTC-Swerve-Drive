package org.firstinspires.ftc.teamcode.modules.jewelswatter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.JewelSwatterConfig;

public class JewelSwatter {
    private JewelArm left, right;
    private JewelSwatterState state;

    public JewelSwatter(HardwareMap hardwareMap) {
        this.left = new JewelArm(hardwareMap, JewelSwatterConfig.LEFT);
        this.right = new JewelArm(hardwareMap, JewelSwatterConfig.RIGHT);
        this.state = JewelSwatterState.STOW_ALL;
    }

    // TODO: add restricttions to states using switch statements
    public int requestState(JewelSwatterState newState) {
        this.state = newState;
        update();
        return 0;
    }

    public void update() {
        switch (state) {
            case DOCK_LEFT:
                left.requestState(JewelArm.JewelArmState.DOCK);
                right.requestState(JewelArm.JewelArmState.STOW);
                break;
            case DOCK_RIGHT:
                left.requestState(JewelArm.JewelArmState.STOW);
                right.requestState(JewelArm.JewelArmState.DOCK);
                break;
            case DOCK_ALL:
                left.requestState(JewelArm.JewelArmState.DOCK);
                right.requestState(JewelArm.JewelArmState.DOCK);
                break;
            case POSSESS_LEFT:
                left.requestState(JewelArm.JewelArmState.POSSESS);
                right.requestState(JewelArm.JewelArmState.STOW);
                break;
            case POSSESS_RIGHT:
                left.requestState(JewelArm.JewelArmState.STOW);
                right.requestState(JewelArm.JewelArmState.POSSESS);
                break;
            case POSSESS_ALL:
                left.requestState(JewelArm.JewelArmState.POSSESS);
                right.requestState(JewelArm.JewelArmState.POSSESS);
                break;
            case RIDE_LOW_LEFT:
                left.requestState(JewelArm.JewelArmState.RIDE_LOW);
                right.requestState(JewelArm.JewelArmState.STOW);
                break;
            case RIDE_LOW_RIGHT:
                left.requestState(JewelArm.JewelArmState.STOW);
                right.requestState(JewelArm.JewelArmState.RIDE_LOW);
                break;
            case RIDE_LOW_ALL:
                left.requestState(JewelArm.JewelArmState.RIDE_LOW);
                right.requestState(JewelArm.JewelArmState.RIDE_LOW);
                break;
            case RIDE_HIGH_LEFT:
                left.requestState(JewelArm.JewelArmState.RIDE_HIGH);
                right.requestState(JewelArm.JewelArmState.STOW);
                break;
            case RIDE_HIGH_RIGHT:
                left.requestState(JewelArm.JewelArmState.STOW);
                right.requestState(JewelArm.JewelArmState.RIDE_HIGH);
                break;
            case RIDE_HIGH_ALL:
                left.requestState(JewelArm.JewelArmState.RIDE_HIGH);
                right.requestState(JewelArm.JewelArmState.RIDE_HIGH);
                break;
            default:
                left.requestState(JewelArm.JewelArmState.STOW);
                right.requestState(JewelArm.JewelArmState.STOW);
        }

        left.update();
        right.update();
    }

    public String getStatus() {
        return state.name();
    }

    public enum JewelSwatterState {
        DOCK_LEFT, POSSESS_LEFT, RIDE_LOW_LEFT, RIDE_HIGH_LEFT,
        DOCK_RIGHT, POSSESS_RIGHT, RIDE_LOW_RIGHT, RIDE_HIGH_RIGHT,
        STOW_ALL, DOCK_ALL, POSSESS_ALL, RIDE_LOW_ALL, RIDE_HIGH_ALL
    }
}

