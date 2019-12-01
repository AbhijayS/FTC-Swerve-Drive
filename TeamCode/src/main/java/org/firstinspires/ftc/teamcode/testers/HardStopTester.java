package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.hardStops;

@TeleOp(name = "Hard Stop Tester")
public class HardStopTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        hardStops stops = new hardStops(this);
        waitForStart();

        while(opModeIsActive()) {
            if (this.gamepad2.x) {
                stops.setState(hardStops.hardStopState.STOW);
            } else if (this.gamepad2.a) {
                stops.setState(hardStops.hardStopState.PARTIAL_DEPLOY);
            } else if (this.gamepad2.b) {
                stops.setState(hardStops.hardStopState.DEPLOY);
            }
            stops.update();
            telemetry.addData("Stop State: ", stops.getState());
            telemetry.update();
        }
    }

}
