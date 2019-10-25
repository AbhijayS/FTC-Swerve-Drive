package org.firstinspires.ftc.teamcode.modules.sensory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odometry {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private PWMOutput Xencoder, Yencoder;
    private Coordinate coordinate;
    private double encoderXZero, encoderYZero;



    public Odometry(LinearOpMode l){
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        Xencoder = hardwareMap.pwmOutput.get("Xencode");
        Yencoder = hardwareMap.pwmOutput.get("Yencode");
        encoderXZero = 0;
        encoderYZero = 0;
    }

}
