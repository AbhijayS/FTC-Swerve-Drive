package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.modules.elevator.Clamp;
import org.firstinspires.ftc.teamcode.modules.elevator.Elevator;

@TeleOp(name = "TestOp: Elevator")
public class ElevatorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        Elevator elevator = new Elevator(this);
        Clamp clamp = new Clamp(this);
        Gamepad gamepad = new Gamepad(this);

        /*DcMotor liftOne = (DcMotorEx) hardwareMap.dcMotor.get("L1");
        DcMotor liftTwo = (DcMotorEx) hardwareMap.dcMotor.get("L2");

        liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftOne.setDirection(DcMotorSimple.Direction.FORWARD);
        liftTwo.setDirection(DcMotorSimple.Direction.FORWARD);*/

        waitForStart();
        while (opModeIsActive()) {
            elevator.updateByGamepad(gamepad, gamepad.lift);
            telemetry.addData("Clamp Status", clamp.getStatus());
            /*double power = Range.scale(gamepad.lift, -1, 1, -0.25, 0.25);
            liftOne.setPower(power);
            liftTwo.setPower(power);*/
            gamepad.update();
            telemetry.update();
        }
    }

}
