package org.firstinspires.ftc.teamcode.aaaaaaaa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="foundationIntakeTest")
public class CatapultDrivetrainRunner extends LinearOpMode {
    public void runOpMode() {
        CatapultDrivetrain drivetrain = new CatapultDrivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        CRServo bobTheMover = hardwareMap.crservo.get("s1");
        waitForStart();
        while (opModeIsActive()){
            drivetrain.updateByGamepad();
            bob.setPosition(0);
            if (gamepad1.x){
                bobTheMover.setPower(-.3);
                wait(500)
                bobTheMover.setPower(0);
            }
            if (gamepade.y) {
                bobTheMover.setPower(.3);
                wait(500)
                bobTheMover.setPower(0);
            }
        }
    }
}