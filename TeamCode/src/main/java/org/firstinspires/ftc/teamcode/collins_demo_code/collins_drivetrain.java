package org.firstinspires.ftc.teamcode.collins_demo_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="foundationIntakeTest")
public class collins_drivetrain extends LinearOpMode {
    

    public void runOpMode() {
        collins_drivetrain drivetrain = new collins_drivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        CRServo bobTheMover = hardwareMap.crservo.get("s1");
        waitForStart();
        while (opModeIsActive()){
            drivetrain.updateByGamepad();
            /**
             * below:
             * if x pressed, then moves the servo down for half a second.
             * if y pressed, then moves servo up for half a second.
             * all this so you can move the foundation in and out of the building zone.
             */
            if (gamepad1.x){
                bobTheMover.setPower(-.3);
                bobTheMover.setPower(0);
            }
            if (gamepad1.y) {
                bobTheMover.setPower(.3);
                bobTheMover.setPower(0);
            }
        }
    }


}