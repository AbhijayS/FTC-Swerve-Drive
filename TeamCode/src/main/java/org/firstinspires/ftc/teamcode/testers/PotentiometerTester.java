package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.sensory.Potentiometer;

@TeleOp (name = "TestOp: Potentiometer")
public class PotentiometerTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Potentiometer potentiometer = new Potentiometer("P",this);
        waitForStart();
        while (opModeIsActive()){
            int angle = potentiometer.returnAngle();
            telemetry.update();
        }
    }
}
