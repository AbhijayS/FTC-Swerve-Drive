package org.firstinspires.ftc.teamcode.collins_demo_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="collins_drivetrain")
public class collins_drivetrain extends LinearOpMode {
    public void runOpMode() {
        catapultDrivetrain drivetrain = new catapultDrivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor RightMotor = hardwareMap.dcMotor.get("m1");
        DcMotor LeftMotor = hardwareMap.dcMotor.get("m2");
        Servo RightServo = hardwareMap.servo.get("s1");
        Servo LeftServo = hardwareMap.servo.get("s2");
        RightMotor.setPower(0);
        LeftMotor.setPower(0);
        waitForStart();
        while (opModeIsActive()){
            drivetrain.updateByGamepad();
            if (gamepad2.left_bumper){
                RightMotor.setPower(1);
                LeftMotor.setPower(-1);
            }

            if (gamepad2.right_bumper) {
                RightMotor.setPower(-1);
                LeftMotor.setPower(1);
            }
            if (gamepad2.x) {
                LeftServo.setPosition(.65);
                RightServo.setPosition(.16);
            }
            if (gamepad2.y) {
                LeftServo.setPosition(.28);
                RightServo.setPosition(.52);
            }
        }
    }
}