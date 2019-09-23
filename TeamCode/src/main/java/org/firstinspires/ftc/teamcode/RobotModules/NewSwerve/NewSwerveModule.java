package org.firstinspires.ftc.teamcode.RobotModules.NewSwerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewSwerveModule {
    private DcMotorEx driveMotor;
    private Servo turnServo;
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;


    public void status(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }

    public NewSwerveModule(LinearOpMode l, String drivemotor, String turnservo, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotorSimple.Direction direction, Servo.Direction servodirection){
        linearOpMode = l;
        telemetry = linearOpMode.telemetry;
        HardwareMap hardwareMap = l.hardwareMap;

        driveMotor = (DcMotorEx) hardwareMap.dcMotor.get(drivemotor);
        driveMotor.setDirection(direction);
        status("Motor Initialized");

        turnServo = hardwareMap.servo.get(turnservo);
        turnServo.setDirection(servodirection);


    }
}
