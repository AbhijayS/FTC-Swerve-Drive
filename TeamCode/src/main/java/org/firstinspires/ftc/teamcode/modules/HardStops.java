package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;

public class HardStops {
    private LinearOpMode linearOpMode;
    private Servo leftStop;
    private Servo rightStop;
    private HardStopState state;

    public HardStops(LinearOpMode l){
        linearOpMode = l;
        HardwareMap hardwareMap = l.hardwareMap;
        leftStop = hardwareMap.servo.get(UniversalConstants.leftStop);
        rightStop = hardwareMap.servo.get(UniversalConstants.rightStop);
        leftStop.setDirection(Servo.Direction.REVERSE);
        rightStop.setDirection(Servo.Direction.REVERSE);
        state = HardStopState.STOW;
    }

    public void update(){
        this.leftStop.setPosition(state.leftPos);
        this.rightStop.setPosition(state.rightPos);
    }

    public void requestState(HardStopState newState){
        state = newState;
    }

    public HardStopState getState(){
        return state;
    }

    public enum HardStopState {
        STOW(0, 0), DEPLOY(.8, .8), PARTIAL_DEPLOY(.4, .4);

        public double leftPos, rightPos;
        HardStopState(double leftPosition, double rightPosition){
            leftPos = leftPosition;
            rightPos = rightPosition;
        }
    }
}