package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Fast Lift")
public class FastLift extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftOne = hardwareMap.dcMotor.get("L1");
        DcMotor liftTwo = hardwareMap.dcMotor.get("L2");

        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftOne.setDirection(DcMotorSimple.Direction.FORWARD);
        liftTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        liftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad2.left_stick_y;
            liftOne.setPower(power);
            liftTwo.setPower(power);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
//    public double convertToTicks(double inches) {
//        double tickValue = 0;
//        double linear_conversion = inches / 4.0;
//        tickValue = linear_conversion * (1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0;
//        return tickValue;
//    }
}
