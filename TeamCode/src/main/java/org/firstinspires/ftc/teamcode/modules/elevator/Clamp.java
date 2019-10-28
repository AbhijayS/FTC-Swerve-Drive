package org.firstinspires.ftc.teamcode.modules.elevator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;

public class Clamp {
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private CRServo bottom, top;
    public double degreeVal = 225;



    public Clamp(LinearOpMode l) {
        linearOpMode = l;
        telemetry = l.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;
        top = hardwareMap.crservo.get("t");
        bottom = hardwareMap.crservo.get("b");
        top.setDirection(DcMotorSimple.Direction.FORWARD);
        bottom.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double convertDegrees(Double deg2conv) {
        double newdeg = deg2conv * (28.0 / 360.0) * (1.0 / 56.0) * 360;
        return newdeg;
    }

    /*public void moveClamp(double degrees) {
        double servodeg = convertDegrees(degrees);
        double position = servodeg * (1.0/180);
        double posMod = position - (20/180.0);
        top.setPosition(posMod);
        bottom.setPosition(position);
    }*/

    public void updateByGamepad(Gamepad g) {
        double power = Range.clip(g.ry * (1 / Math.abs(g.ry)) * Math.abs(Math.cbrt(g.ry)), -1, 1);
        top.setPower(power);
        bottom.setPower(power);

        /*if (linearOpMode.gamepad2.a) {
           // moveClamp(134); //Increasing makes servo go out with config
            //.3722
            degreeVal = 124;//closing state
        } else if (linearOpMode.gamepad2.b) {
            //moveClamp(215);//position .60001
            degreeVal = 225;

        }
        moveClamp(degreeVal);

        telemetry.addData("Top Position: ", top.getPosition());
        telemetry.addData("Bottom Pos: ",bottom.getPosition());*/
    }
}
