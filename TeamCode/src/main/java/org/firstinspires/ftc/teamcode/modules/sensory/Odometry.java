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
    private double XencoderZero, YencoderZero;
    private ArrayList<Coordinate> snapshot = new ArrayList<Coordinate>(); // This array List will be a snapshot of the last ten seconds of match position
    private Coordinate current;
    private final double CPR = 512;
    private final double wheel = 1.1811; // inches


    public Odometry(LinearOpMode l){
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        Xencoder = hardwareMap.dcMotor.get("Xencode");
        Yencoder = hardwareMap.dcMotor.get("Yencode");
        Xencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Yencoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        XencoderZero = (double) Xencoder.getCurrentPosition();
        YencoderZero = (double) Yencoder.getCurrentPosition();
        telemetry.addData("X Origin: ", XencoderZero);
        telemetry.addData("Y Origin: ", YencoderZero);
        origin = new Coordinate(XencoderZero, YencoderZero);
        current = origin;
    }







}
