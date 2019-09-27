package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotModules.TensorFlowLite;

@TeleOp(name = "TensorFlow Tester")
public class TensorFlowTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        TensorFlowLite tensorFlowLite = new TensorFlowLite(this);

        tensorFlowLite.activateTfod();
        waitForStart();

        while (opModeIsActive()){
            tensorFlowLite.updateTensorFlowSingle();
        }
    }
}
