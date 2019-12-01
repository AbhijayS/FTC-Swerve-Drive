package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;

public class hardStops{
    private LinearOpMode linearOpMode;
    private Servo leftStop;
    private Servo rightStop;
    private hardStopState state;

    public hardStops(LinearOpMode l){
        linearOpMode = l;
        HardwareMap hardwareMap = l.hardwareMap;
        leftStop = hardwareMap.servo.get(UniversalConstants.leftStop);
        rightStop = hardwareMap.servo.get(UniversalConstants.rightStop);
        leftStop.setDirection(Servo.Direction.FORWARD);
        rightStop.setDirection(Servo.Direction.REVERSE);
    }

    public void update(){
        this.leftStop.setPosition(state.leftPos);
        this.rightStop.setPosition(state.rightPos);
    }

    public void setState(hardStopState newState){
        state = newState;
    }

    public hardStopState getState(){
        return state;
    }

    public enum hardStopState{
        STOW(0, 0), DEPLOY(.5, .5), PARTIAL_DEPLOY(.25, .25);

        public double leftPos, rightPos;
        hardStopState(double leftPosition, double rightPosition){
            leftPos = leftPosition;
            rightPos = rightPosition;
        }
    }
}
