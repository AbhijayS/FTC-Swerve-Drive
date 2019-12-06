package org.firstinspires.ftc.teamcode.modules.sensory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Potentiometer {
    private AnalogInput a;
    LinearOpMode linearOpMode;

    public Potentiometer(String name, LinearOpMode linearOpMode){
        this.linearOpMode = linearOpMode;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        a = hardwareMap.analogInput.get(name);
    }

    public int returnAngle(){
        double voltage = a.getVoltage();
        int angle = (int) Math.rint(voltage*81.9423368741);
        //linearOpMode.telemetry.addData("Voltage: ",voltage);
        linearOpMode.telemetry.addData("Pot Angle: ",angle);
        return angle;
    }
}
