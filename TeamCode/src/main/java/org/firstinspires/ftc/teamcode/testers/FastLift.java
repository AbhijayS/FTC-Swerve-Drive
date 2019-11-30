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
        double power;
//        int tp;

        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftOne.setDirection(DcMotorSimple.Direction.FORWARD);
        liftOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to go!");

        waitForStart();

        power = 0.05;
//        tp = (int) convertToTicks(5);
        liftOne.setPower(power);
//        liftOne.setTargetPosition(tp);

        while (opModeIsActive()) {
            idle();
        }
    }

    public double convertToTicks(double inches) {
        double tickValue = 0;
        double linear_conversion = inches / 4.0;
        tickValue = linear_conversion * (1.0 / (2.3 * Math.PI)) * 62 * (38.0 / 62.0) * (1.0 / 38.0) * 560.0;
        return tickValue;
    }
}
