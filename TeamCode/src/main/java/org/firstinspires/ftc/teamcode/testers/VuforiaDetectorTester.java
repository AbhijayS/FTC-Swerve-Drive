package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.sensory.VuforiaDetector;

@TeleOp(name = "DEBUGGING: Stone Detector")
public class VuforiaDetectorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaDetector vuDetector = new VuforiaDetector(hardwareMap);

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Blue Pattern", vuDetector.getPatternBlue());
            telemetry.addData("Red Pattern", vuDetector.getPatternRed());
            telemetry.update();
            idle();
        }
    }
}
