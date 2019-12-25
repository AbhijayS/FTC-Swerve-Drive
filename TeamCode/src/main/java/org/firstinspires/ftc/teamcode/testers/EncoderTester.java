package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.driveGearRatio;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.oppositeSigns;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ticksPerRevolution;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.wheelCircumference;

@TeleOp(name = "TestOp: Encoder Tester")
public class EncoderTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get(UniversalConstants.ModuleConfig.MODULE_ZERO.motorID);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Pos", convertTicksToInches(motor.getCurrentPosition()));
            telemetry.update();
        }
    }
    private double convertTicksToInches(double ticks) {
        return ticks * wheelCircumference * driveGearRatio / ticksPerRevolution;
    }
}
