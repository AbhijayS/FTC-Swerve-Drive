package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private Servo bottom, top;


    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        top = hardwareMap.servo.get("t");
        bottom = hardwareMap.servo.get("b");
        top.setDirection(Servo.Direction.FORWARD);
        bottom.setDirection(Servo.Direction.FORWARD);

    }

    public double convertDegrees(Double deg2conv) {
        double newdeg = deg2conv * (28.0 / 360.0) * (1.0 / 56.0) * 360;
        return newdeg;
    }

    public void moveClamp(double degrees) {
        double servodeg = convertDegrees(degrees);
        double position = servodeg * (1.0/180);
        top.setPosition(position);
        bottom.setPosition(position);
    }

    public void updateByGampad() {
        if (linearOpMode.gamepad2.a) {
            moveClamp(134); //Increasing makes servo go out with config
            //.3722
        } else if (linearOpMode.gamepad2.b) {
            moveClamp(215);//position .60001
        }
        telemetry.addData("Top Position: ", top.getPosition());
        telemetry.addData("Bottom Pos: ",bottom.getPosition());

    }
}
