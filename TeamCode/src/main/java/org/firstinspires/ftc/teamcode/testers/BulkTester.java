package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.RevBulkData;

@TeleOp(name = "Bulk Loop Times")
public class BulkTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ExpansionHubEx expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        RevBulkData bulkData;
        DcMotor motor0 = hardwareMap.dcMotor.get("motor0");
        DcMotor motor1 = hardwareMap.dcMotor.get("motor1");
        DcMotor motor2 = hardwareMap.dcMotor.get("motor2");
        DcMotor motor3 = hardwareMap.dcMotor.get("motor3");

        final int cycles = 100000;
        Stopwatch stopwatch = new Stopwatch();
        int counter;
        double elapsed;
        double period;
        double hz;

        telemetry.addLine("Ready to go!");
        telemetry.update();

        waitForStart();

        ///////////////////////////////////////////
        // Test 1.1:
        // Get encoder value for 1 DcMotor normally
        ///////////////////////////////////////////

        stopwatch.reset();
        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            motor0.getCurrentPosition();
            counter++;
            idle();
        }

        stopwatch.stop();
        elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        period = elapsed / cycles; // delay between each cycle
        hz = 1000 / period; // number of cycles per second

        telemetry.addData("Test 1.1", "Get encoder value for 1 DcMotor normally");
        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.addLine();

        ////////////////////////////////////////////
        // Test 1.2:
        // Get encoder value for 2 DcMotors normally
        ////////////////////////////////////////////

        stopwatch.reset();
        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            motor0.getCurrentPosition();
            motor1.getCurrentPosition();
            counter++;
            idle();
        }

        stopwatch.stop();
        elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        period = elapsed / cycles; // delay between each cycle
        hz = 1000 / period; // number of cycles per second

        telemetry.addData("Test 1.2", "Get encoder value for 2 DcMotors normally");
        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.addLine();

        ////////////////////////////////////////////////
        // Test 1.3:
        // Get encoder value for ALL 4 DcMotors normally
        ////////////////////////////////////////////////

        stopwatch.reset();
        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            motor0.getCurrentPosition();
            motor1.getCurrentPosition();
            motor2.getCurrentPosition();
            motor3.getCurrentPosition();
            counter++;
            idle();
        }

        stopwatch.stop();
        elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        period = elapsed / cycles; // delay between each cycle
        hz = 1000 / period; // number of cycles per second

        telemetry.addData("Test 1.3", "Get encoder value for ALL 4 DcMotors normally");
        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.addLine();



        ///////////////////////////////////////
        // Test 2.1:
        // Get Bulk encoder value for 1 DcMotor
        ///////////////////////////////////////

        stopwatch.reset();
        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            bulkData = expansionHub.getBulkInputData();
            bulkData.getMotorCurrentPosition(motor0);
            counter++;
            idle();
        }

        stopwatch.stop();
        elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        period = elapsed / cycles; // delay between each cycle
        hz = 1000 / period; // number of cycles per second

        telemetry.addData("Test 2.1", "Get Bulk encoder value for 1 DcMotor");
        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.addLine();

        ////////////////////////////////////////
        // Test 2.2:
        // Get Bulk encoder value for 2 DcMotors
        ////////////////////////////////////////

        stopwatch.reset();
        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            bulkData = expansionHub.getBulkInputData();
            bulkData.getMotorCurrentPosition(motor0);
            bulkData.getMotorCurrentPosition(motor1);
            counter++;
            idle();
        }

        stopwatch.stop();
        elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        period = elapsed / cycles; // delay between each cycle
        hz = 1000 / period; // number of cycles per second

        telemetry.addData("Test 2.2", "Get Bulk encoder value for 2 DcMotor2");
        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.addLine();

        ////////////////////////////////////////////
        // Test 2.3:
        // Get Bulk encoder value for ALL 4 DcMotors
        ////////////////////////////////////////////

        stopwatch.reset();
        counter = 0;
        stopwatch.start();

        while (opModeIsActive() && counter < cycles) {
            bulkData = expansionHub.getBulkInputData();
            bulkData.getMotorCurrentPosition(motor0);
            bulkData.getMotorCurrentPosition(motor1);
            bulkData.getMotorCurrentPosition(motor2);
            bulkData.getMotorCurrentPosition(motor3);
            counter++;
            idle();
        }

        stopwatch.stop();
        elapsed = stopwatch.millis(); // total elapsed time for 100,000 cycles
        period = elapsed / cycles; // delay between each cycle
        hz = 1000 / period; // number of cycles per second

        telemetry.addData("Test 2.3", "Get Bulk encoder value for ALL 4 DcMotors");
        telemetry.addData(cycles + " cycles", elapsed + " ms");
        telemetry.addData("Period", period + " ms");
        telemetry.addData("Hz", hz);
        telemetry.addLine();

        ////////////////////////////////////////////
        ////////////////////////////////////////////

        telemetry.addLine("All done!");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
