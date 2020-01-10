package org.firstinspires.ftc.teamcode.opmodes.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;

import java.util.ArrayList;

@Autonomous(name = "Just Clamp")
public class BlueClampPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Clamp clamp = new Clamp(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            clamp.requestState(Clamp.ClampState.CLAMP);
            clamp.update();
            idle();
        }

    }
}
