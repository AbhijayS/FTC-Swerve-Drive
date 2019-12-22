package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.UniversalConstants;
import org.firstinspires.ftc.teamcode.common.utilities.Debugger;
import org.firstinspires.ftc.teamcode.common.utilities.Gamepad;
import org.firstinspires.ftc.teamcode.common.utilities.Util;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.modules.swerve.SwerveModule;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_ONE;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_THREE;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_TWO;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ModuleConfig.MODULE_ZERO;

@TeleOp(name = "TestOp: New Driving")
public class NewDriving extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g = new Gamepad(this);
        SwerveDrive swerve = new SwerveDrive(this, null);
        waitForStart();
        while (opModeIsActive()) {
            swerve.fod(g);
            g.update();
            telemetry.update();
            idle();
        }
    }
}
