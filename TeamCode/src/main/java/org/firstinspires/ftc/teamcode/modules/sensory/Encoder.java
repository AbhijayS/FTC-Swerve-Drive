package org.firstinspires.ftc.teamcode.modules.sensory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Encoder {
    private I2cDevice device;
    private int CPR = 360;
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;

    public Encoder(LinearOpMode l, String name){
        linearOpMode = l;
        telemetry = l.telemetry;

    }

}
