package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.utilities.Stopwatch;
import org.firstinspires.ftc.teamcode.modules.sensory.Gyro;
import org.firstinspires.ftc.teamcode.modules.sensory.Odometry;

public class OdometryTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new Odometry(this);
        Gyro gyro = new Gyro(this);
        Stopwatch stopwatch = new Stopwatch();
        double time = 0; //time in seconds
        double heading;

        waitForStart();
        stopwatch.start();
        while (opModeIsActive()){
        time = stopwatch.seconds();
        heading = gyro.getHeading();

        }
    }
}
