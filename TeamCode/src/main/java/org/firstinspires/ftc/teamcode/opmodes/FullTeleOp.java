package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Elevator;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;

@TeleOp (name = "Opmode: TeleOp")
public class FullTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive swerveDrive = new SwerveDrive(this, null);
        Clamp clamp = new Clamp(this);
        Gamepad gamepad = new Gamepad(this);
        gamepad.update();

        DcMotor liftOne = (DcMotorEx) hardwareMap.dcMotor.get("L1");
        DcMotor liftTwo = (DcMotorEx) hardwareMap.dcMotor.get("L2");

        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftOne.setDirection(DcMotorSimple.Direction.FORWARD);
        liftTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            clamp.updateByGamepad(gamepad);
            telemetry.addData("Clamp Status", clamp.getStatus());

            double power = gamepad.lift;
            if (liftTwo.getCurrentPosition() <= 0 && power < 0) {
                power = 0;
            }
            liftOne.setPower(power);
            liftTwo.setPower(power);

            telemetry.addData("Position", liftTwo.getCurrentPosition());
            swerveDrive.fod(gamepad);
//            swerveDrive.swerveKinematics.update();
            gamepad.update();
            telemetry.update();
        }
    }
}
