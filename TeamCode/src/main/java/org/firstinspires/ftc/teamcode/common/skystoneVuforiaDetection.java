package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class skystoneVuforiaDetection {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;

    public skystoneVuforiaDetection(LinearOpMode l){
        linearOpMode = l;
        telemetry = linearOpMode.telemetry;
    }
}
