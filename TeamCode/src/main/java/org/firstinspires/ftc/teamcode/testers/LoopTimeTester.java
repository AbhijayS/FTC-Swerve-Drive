package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;

@TeleOp(name = "Raw Loop Times")
public class LoopTimeTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Stopwatch stopwatch = new Stopwatch();
        final int cycles = 100000;
        int counter;

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            counter++;
            idle();
        }

        stopwatch.stop();
        double elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        double period = elapsed / cycles; // delay between each cycle
        double hz = 1000 / period; // number of cycles per second

        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
