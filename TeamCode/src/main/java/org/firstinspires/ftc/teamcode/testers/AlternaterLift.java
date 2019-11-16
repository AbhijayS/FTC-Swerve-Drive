package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.elevator.LiftModule;

public class AlternaterLift extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LiftModule liftModule = new LiftModule(this);
        waitForStart();
        while (opModeIsActive()){
            liftModule.alternateState();
        }
    }
}
