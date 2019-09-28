package org.firstinspires.ftc.teamcode.collins_demo_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="collins_drivetrain")
public class Test extends LinearOpMode {
    public void runOpMode() {
//        catapultDrivetrain drivetrain = new catapultDrivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        DcMotor RightMotor = hardwareMap.dcMotor.get("m1");
        DcMotor LeftMotor = hardwareMap.dcMotor.get("m2");
        DcMotor leftFront = hardwareMap.dcMotor.get("lf");
        DcMotor rightFront = hardwareMap.dcMotor.get("rf");
        Servo RightServo = hardwareMap.servo.get("s1");
        Servo LeftServo = hardwareMap.servo.get("s2");
        RightMotor.setPower(0);
        LeftMotor.setPower(0);
        double scale = 0.25;
        waitForStart();
        while (opModeIsActive()){

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

            double leftPower = scale*gamepad1.left_trigger;
            double rightPower = -1*scale*gamepad1.right_trigger;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);

        }
    }
}