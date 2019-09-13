package org.firstinspires.ftc.teamcode.aaaaaaaa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Test")
public class CatapultDrivetrainRunner extends LinearOpMode {
    public void runOpMode() {
        CatapultDrivetrain drivetrain = new CatapultDrivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()){
            drivetrain.updateByGamepad();
        }
    }
}