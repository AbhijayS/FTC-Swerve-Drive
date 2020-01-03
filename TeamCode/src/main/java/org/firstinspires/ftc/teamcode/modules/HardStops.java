package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

public class HardStops {
    private static final double STOW = 0;
    private static final double HORIZONTAL = 0.33;
    private static final double PROTECT = 0.5;
    private Servo leftStop;
    private Servo rightStop;
    private HardStopState state;

    public HardStops(HardwareMap hardwareMap) {
        this.leftStop = hardwareMap.servo.get(UniversalConstants.leftStop);
        this.rightStop = hardwareMap.servo.get(UniversalConstants.rightStop);
        this.leftStop.setDirection(Servo.Direction.REVERSE);
        this.rightStop.setDirection(Servo.Direction.REVERSE);
        requestState(HardStopState.STOW_ALL);
        update();
    }

    public void update() {
        this.leftStop.setPosition(state.leftPos);
        this.rightStop.setPosition(state.rightPos);
    }

    public void updateUsingGamepad(Gamepad gamepad) {
    }

    public void requestState(HardStopState newState) {
        state = newState;
    }

    public HardStopState getState() {
        return state;
    }

    public enum HardStopState {

        // left stop,
        HORIZONTAL_LEFT(HORIZONTAL, STOW),
        PROTECT_LEFT(PROTECT, STOW),

        // right stop,
        HORIZONTAL_RIGHT(STOW, HORIZONTAL),
        PROTECT_RIGHT(STOW, PROTECT),

        // misc
        STOW_ALL(STOW, STOW),
        DEFENSE(HORIZONTAL, HORIZONTAL),
        DRIVE(PROTECT, PROTECT);

        public double leftPos, rightPos;

        HardStopState(double leftPosition, double rightPosition) {
            leftPos = leftPosition;
            rightPos = rightPosition;
        }
    }
}