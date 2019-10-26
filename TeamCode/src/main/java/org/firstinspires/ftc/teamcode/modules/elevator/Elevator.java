package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Elevator {
    private LinearOpMode linearOpMode;
    Clamp clamp;
    LiftModule liftModule;

    public Elevator(LinearOpMode l){
        linearOpMode = l;
        clamp = new Clamp(l);
    }

    public void updateByGamepad(){
        clamp.updateByGampad();
        linearOpMode.telemetry.update();
    }
}