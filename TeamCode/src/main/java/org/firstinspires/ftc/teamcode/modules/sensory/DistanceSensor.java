package org.firstinspires.ftc.teamcode.modules.sensory;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private ModernRoboticsI2cRangeSensor distSensor;
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;

    public DistanceSensor(LinearOpMode l, String name){
        linearOpMode = l;
        telemetry = linearOpMode.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;

        distSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, name);
    }

    public double getDistanceUltra(){
        return distSensor.cmUltrasonic();
    }

    public double getDistanceHybrid(){return distSensor.getDistance(DistanceUnit.CM);}


    public void updateTelemetry(){
        telemetry.addData("Measured distance Ultrasonic: ", getDistanceUltra());

        telemetry.update();
    }

}
