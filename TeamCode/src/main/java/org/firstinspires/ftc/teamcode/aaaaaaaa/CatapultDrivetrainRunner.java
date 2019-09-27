package org.firstinspires.ftc.teamcode.aaaaaaaa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.VEX393Servo;

@TeleOp(name="foundationIntakeTest")
public class CatapultDrivetrainRunner extends LinearOpMode {
    public void runOpMode() {
        CatapultDrivetrain drivetrain = new CatapultDrivetrain(this, DcMotor.ZeroPowerBehavior.BRAKE);
        Servo foundationMover = hardwareMap.servo.get("s1");
        waitForStart();
        while (opModeIsActive()){
            drivetrain.updateByGamepad();
            servoCheck();
            /**
             * below:
             * if x pressed, then moves the servo down for half a second.
             * if y pressed, then moves servo up for half a second.
             * all this so you can move the foundation in and out of the building zone.
             *
            if (gamepad1.x){
                foundationMover.setPosition(1);
            }
            if (gamepad1.y) {
                foundationMover.setPosition(1);
            }
<<<<<<< Updated upstream
             **/

        }
    }

    private void servoCheck() {
        float tgtPower = -this.gamepad1.left_stick_y;
        VEX393Servo motorTest = null;
        motorTest.setPower(tgtPower);
        // check to see if we need to move the servo.
        Servo servoTest = null;
        if(gamepad1.y) {
            // move to 0 degrees.
            servoTest.setPosition(0);
        } else if (gamepad1.x) {
            // move to 90 degrees.
            servoTest.setPosition(0.5);
        } else if (gamepad1.left_bumper) {
            // move to 180 degrees.
            servoTest.setPosition(1);
        }
        telemetry.addData("Servo Position", servoTest.getPosition());
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", motorTest.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}