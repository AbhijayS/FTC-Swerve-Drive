package org.firstinspires.ftc.teamcode.collins_demo_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="collins_drivetrain")
public class collins_drivetrain extends LinearOpMode {
    

    public void runOpMode() {
        collins_drivetrain drivetrain = new collins_drivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        Motor RightMotor = hardwareMap.Motor.get("m1");
        Motor LeftMotor = hardwareMap.Motor.get("m2");
        Servo RightServo = hardwareMap.Servo.get("s1");
        Servo LeftServo = hardwareMap.Servo.get("s2");
        waitForStart();
        while (opModeIsActive()){
            drivetrain.updateByGamepad();
            if (gamepad1.Trigger1 > 0){
                RightMotor.setPower(1);
                LeftMotor.setPower(-1);
            }
            if (gamepad1.Trigger1 = 0) {
                RightMotore.setPower(0);
                LeftMotor.setPower(0);
            }
            if (gamepad1.x = 1) {
                RightServo.power(1);
                LeftServo.power(1);
            }
            if (gamepad.y = 1) {
                RightServo.power(0);
                LeftServo.power(0);
            }
        }
    }


}