package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TestOp: Lift")
public class LiftTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftMotor = hardwareMap.dcMotor.get("lift");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double position = 1400;
        int count = -1;

        waitForStart();
        liftMotor.setPower(.75);
        while (opModeIsActive()) {
            if (count <= 100){
                if (Math.abs(liftMotor.getTargetPosition() - liftMotor.getCurrentPosition()) < 75) {
                    if (position == 1400) {
                        liftMotor.setPower(1);
                        position = 0;
                        count = count + 1;
                    } else {
                        position = 1400;
                        liftMotor.setPower(1);
                    }
                    Thread.sleep(500);
                    liftMotor.setTargetPosition((int) position);
                }
                telemetry.addData("times lifted: ", count);
                telemetry.addData("Position: ", liftMotor.getCurrentPosition());
                telemetry.addData("Target Position: ", liftMotor.getTargetPosition());
                telemetry.update();
            } else {
                telemetry.addLine("100 lifts successful");
                telemetry.update();
            }
//            while (counter < 1 && opModeIsActive()) {
//                ElapsedTime elapsedTime = new ElapsedTime();
//                //while(elapsedTime.time() < 100000000 && opModeIsActive()){
//                    liftModule.moveHeight(2000, 0.5);
//                    telemetry.addData("Lift Status: ", "Going Up!");
//                    telemetry.addData("counter: ", counter);
//                    telemetry.addData("Time", elapsedTime.time());
//                    telemetry.update();
//                //}
////                while(elapsedTime.time() < 1000000&& opModeIsActive()){
////                    liftModule.moveHeight(0, 0.5);
////                    telemetry.addData("Lift Status: ", "Going Down!");
////                    telemetry.addData("counter: ", counter);
////                    telemetry.update();
////                }
//                //counter = counter + 1;
//                telemetry.addData("counter: ", counter);
//                telemetry.update();
//            }
//            telemetry.addData("lift status: ", "program over");
//            telemetry.update();
        }
    }
}
