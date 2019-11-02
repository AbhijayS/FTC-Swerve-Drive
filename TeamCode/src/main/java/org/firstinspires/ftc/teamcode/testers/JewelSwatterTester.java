package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.jewelswatter.JewelSwatter;

@TeleOp(name = "TestOp: Jewel Swatter")
public class JewelSwatterTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        JewelSwatter jewelSwatter = new JewelSwatter(this);
        waitForStart();
        while (opModeIsActive()) {
            //sets the positions if you press x
            if (gamepad2.x) {
                telemetry.addData("Status", "Pre-Clamp");
                jewelSwatter.preclampRight();
                jewelSwatter.preclampLeft();
            }
            //sets the positions if you press y
            if (gamepad2.y) {
                telemetry.addData("Status", "Clamp");
                jewelSwatter.clampRight();
                jewelSwatter.clampLeft();
            }
            if (gamepad2.a) {
                telemetry.addData("Status", "Stow");
                jewelSwatter.stowRight();
                jewelSwatter.stowLeft();
            }

            if (gamepad2.b) {
                telemetry.addData("Status", "Travelling High");
                jewelSwatter.travellingLeftHigh();
                jewelSwatter.travellingRightHigh();
            }

            if (gamepad2.dpad_up){
                telemetry.addData("Status", "Travelling Low");
                jewelSwatter.travellingLeftLow();
                jewelSwatter.travellingRightLow();
            }
            telemetry.update();
        }
    }
}
