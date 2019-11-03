package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.sensory.TensorFlowLite;

@TeleOp(name = "TensorFlow Tester")
public class TensorFlowTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TensorFlowLite tensorFlowLite = new TensorFlowLite(this,.45);

        tensorFlowLite.activateTfod();
        waitForStart();

        while (opModeIsActive()){
            tensorFlowLite.twoStone();
        }
        tensorFlowLite.shutDownTfod();
    }
}
