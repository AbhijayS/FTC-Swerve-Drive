package org.firstinspires.ftc.teamcode.modules.sensory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Odometry {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private DcMotor Xencoder, Yencoder;
    private Coordinate origin;
    private double encoderXZero, encoderYZero;
    private ArrayList<Coordinate> snapshot = new ArrayList<Coordinate>(); // This array List will be a snapshot of the last ten seconds of match position


    public Odometry(LinearOpMode l){
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        Xencoder = hardwareMap.dcMotor.get("Xencode");
        Yencoder = hardwareMap.dcMotor.get("Yencode");
        Xencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Yencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderXZero = (double) Xencoder.getCurrentPosition();
        encoderYZero = (double) Yencoder.getCurrentPosition();
        origin = new Coordinate(encoderXZero,encoderYZero);
    }



}
