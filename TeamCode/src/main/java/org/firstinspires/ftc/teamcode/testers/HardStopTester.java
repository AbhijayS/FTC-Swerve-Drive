package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.HardStops;

@TeleOp(name = "Hard Stop Tester")
public class HardStopTester extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        HardStops stops = new HardStops(this);
        waitForStart();

        while(opModeIsActive()) {
            if (this.gamepad2.x) {
                stops.requestState(HardStops.HardStopState.STOW);
            } else if (this.gamepad2.a) {
                stops.requestState(HardStops.HardStopState.PARTIAL_DEPLOY);
            } else if (this.gamepad2.b) {
                stops.requestState(HardStops.HardStopState.DEPLOY);
            }
            stops.update();
            telemetry.addData("Stop State: ", stops.getState());
            telemetry.update();
        }
    }

}
