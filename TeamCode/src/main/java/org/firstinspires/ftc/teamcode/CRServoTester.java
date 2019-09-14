package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@Disabled
@TeleOp(name = "CRServoTester")
public class CRServoTester extends LinearOpMode {
    CRServo servoL;
    @Override
    public void runOpMode() throws InterruptedException {
        servoL = hardwareMap.crservo.get("s1");
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.a){
                servoL.setPower(.3);
            }
            telemetry.addData("Left", servoL.getPower());
            telemetry.update();
        }
    }
}
