package org.firstinspires.ftc.teamcode.RobotModules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.pwmRange;
import static org.firstinspires.ftc.teamcode.RobotModules.UniversalConstants.servoDefaultAngle;

public class CastorModule {
    private DcMotorEx driveMotor;
    private Servo angleservo;
    private LinearOpMode linearOpMode;
    private Telemetry telemetry;
    private UniversalConstants.ModuleConfig module;
    private double servoAngle;

    public CastorModule(LinearOpMode l, UniversalConstants.ModuleConfig moduleConfig, DcMotorSimple.Direction direction){
        module = moduleConfig;
        linearOpMode = l;
        telemetry = l.telemetry;

        angleservo = linearOpMode.hardwareMap.get(Servo.class, module.servoID);
        angleservo.setDirection(module.servoDirection);

        driveMotor = (DcMotorEx) linearOpMode.hardwareMap.get(module.motorID);
        driveMotor.setDirection(direction);

        if (angleservo.getController() instanceof ServoControllerEx) {
            // Confirm its an extended range servo controller before we try to set to avoid crash
            ServoControllerEx theControl = (ServoControllerEx) angleservo.getController();
            theControl.setServoPwmRange(angleservo.getPortNumber(), pwmRange);
        }

        servoAngle = servoDefaultAngle;


    }
}
