package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.sensory.Gyro;
import org.firstinspires.ftc.teamcode.modules.sensory.Odometry;

public class OdometryTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Odometry odometry = new Odometry(this);
        Gyro gyro = new Gyro(this);
        waitForStart();
        while (opModeIsActive()){

        }
    }
}
