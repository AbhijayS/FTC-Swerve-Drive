package org.firstinspires.ftc.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Quick Proto")
public class SixWheelPrototype extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        DcMotor Rone = hardwareMap.dcMotor.get("R1");
        DcMotor Rtwo = hardwareMap.dcMotor.get("R2");
        //DcMotor Rthree = hardwareMap.dcMotor.get("R3");
        Rone.setDirection(DcMotorSimple.Direction.REVERSE);
        Rtwo.setDirection(DcMotorSimple.Direction.REVERSE);
        //Rthree.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor one = hardwareMap.dcMotor.get("L1");
        DcMotor two = hardwareMap.dcMotor.get("L2");
        //DcMotor three = hardwareMap.dcMotor.get("L3");
        one.setDirection(DcMotorSimple.Direction.FORWARD);
        two.setDirection(DcMotorSimple.Direction.FORWARD);
        //three.setDirection(DcMotorSimple.Direction.FORWARD);
        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rone.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()){
            double powerL = gamepad1.left_stick_y;
            double powerR = gamepad1.right_stick_y;
            double Ltrig = gamepad1.left_trigger;
            double Rtrig = gamepad1.right_trigger;
            one.setPower(powerL);

            //three.setPower(powerL);
            Rone.setPower(powerR);
            if(Ltrig  > 0) {
                two.setPower(-Ltrig);
                Rtwo.setPower(-Ltrig);
            }else {
                two.setPower(Rtrig);
                Rtwo.setPower(Rtrig);
            }

            //Rthree.setPower(powerR);
        }
    }
}
