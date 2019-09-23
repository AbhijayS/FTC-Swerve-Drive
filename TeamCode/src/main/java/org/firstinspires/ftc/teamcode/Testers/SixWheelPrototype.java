package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Six Wheel Tester")
public class SixWheelPrototype extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor Rone = hardwareMap.dcMotor.get("R1");
        DcMotor Rtwo = hardwareMap.dcMotor.get("R2");
        DcMotor Rthree = hardwareMap.dcMotor.get("R3");
        Rone.setDirection(DcMotorSimple.Direction.FORWARD);
        Rtwo.setDirection(DcMotorSimple.Direction.FORWARD);
        Rthree.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor one = hardwareMap.dcMotor.get("L1");
        DcMotor two = hardwareMap.dcMotor.get("L2");
        DcMotor three = hardwareMap.dcMotor.get("L3");
        one.setDirection(DcMotorSimple.Direction.REVERSE);
        two.setDirection(DcMotorSimple.Direction.REVERSE);
        three.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()){
            double powerL = gamepad1.left_stick_y;
            double powerR = gamepad1.right_stick_y;
            one.setPower(powerL);
            two.setPower(powerL);
            three.setPower(powerL);
            Rone.setPower(powerR);
            Rtwo.setPower(powerR);
            Rthree.setPower(powerR);
        }
    }
}
