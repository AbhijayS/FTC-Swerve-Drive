package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.sensory.TensorFlowLite;


@TeleOp (name = "TensorFlow Tester Blue")
public class TensorFlowTesterB extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TensorFlowLite tensorFlowLite = new TensorFlowLite(this,.28);

        tensorFlowLite.activateTfod();
        waitForStart();

        while (opModeIsActive()){
            tensorFlowLite.twoStoneB();
        }
        tensorFlowLite.shutDownTfod();
    }
}
