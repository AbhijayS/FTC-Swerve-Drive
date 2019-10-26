package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

public class Elevator {
    private LinearOpMode linearOpMode;
    Clamp clamp;
    LiftModule liftModule;

    public Elevator(LinearOpMode l){
        linearOpMode = l;
        //clamp = new Clamp(l);
        liftModule = new LiftModule(l);
    }

    public void updateByGamepad(Gamepad g){
        //clamp.updateByGamepad();
        liftModule.updateByGamepad(g);
        linearOpMode.telemetry.update();
    }
}
