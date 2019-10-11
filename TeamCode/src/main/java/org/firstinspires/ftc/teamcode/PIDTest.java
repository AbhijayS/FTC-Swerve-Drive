package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestOP: PID")
public class PIDTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rev20 = hardwareMap.get(DcMotor.class, "m");
        rev20.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rev20.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rev20.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        int ticksPerRev = 560;
        double setAngle = 90; // degrees
        double maxSpeed = 0.5;
        double kP = maxSpeed/90;

        double setTicks = (ticksPerRev/360.0)*setAngle;

        waitForStart();

        while (opModeIsActive()) {
            // Manual proportional
            double ticks = rev20.getCurrentPosition();
            double power = Range.clip((setTicks-ticks)*kP, -maxSpeed, maxSpeed);
            rev20.setPower(power);


            // Hub
//            rev20.setTargetPosition((int)setTicks);
        }

    }
}
