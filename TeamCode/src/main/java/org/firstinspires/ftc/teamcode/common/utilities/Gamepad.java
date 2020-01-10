package org.firstinspires.ftc.teamcode.common.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

// TODO: Port all gamepad commands here
public class Gamepad {
    private com.qualcomm.robotcore.hardware.Gamepad driver;
    private com.qualcomm.robotcore.hardware.Gamepad operator;

    private LinearOpMode linearOpMode;
    // driver controls
    public boolean _0, _45, _90, _135, _180, _225, _270, _315, heading, Oa, Ob, Ou, Ol, Or, Od, slowmo;
    public boolean fod = false;
    public boolean reset_gyro = false;
    public boolean stowHardStops,alignRight,alignLeft,drive;

    // operator controls
    public boolean clamp;
    public boolean approach;
    public boolean partial;
    public boolean clampStowed;
    public double lift;
    public boolean stowLift, extendLift, incrementLift, decrementLift;
    private Stopwatch liftWatch = new Stopwatch();
    public double microAdjustLift;

    public double x, y, z, lx, ly, rx, ry;

    public Gamepad(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.driver = linearOpMode.gamepad1;
        this.operator = linearOpMode.gamepad2;
        this.liftWatch.start();
        update();
    }

    public void update() {

        /* Driver Commands*/
        _0 = driver.dpad_right;
        _90 = driver.dpad_up;
        _180 = driver.dpad_left;
        _270 = driver.dpad_down;
        slowmo = driver.left_bumper;

        if (driver.left_trigger >= 0)
            reset_gyro = true;
        if (driver.left_trigger == 0)
            reset_gyro = false;

        heading = (_0 || _90 || _180 || _270) && !(_0 && _90 && _180 && _270);

        _45 = _0 && _90;
        _135 = _90 && _180;
        _225 = _180 && _270;
        _315 = _270 && _0;

        x = driver.left_stick_x;
        y = -driver.left_stick_y;
        z = driver.right_stick_x;

        if (driver.y) {
            stowHardStops = true;
            alignRight = false;
            alignLeft  = false;
            drive = false;
        }

        if (driver.b) {
            stowHardStops = false;
            alignRight = true;
            alignLeft  = false;
            drive = false;
        }

        if (driver.x) {
            stowHardStops = false;
            alignRight = false;
            alignLeft  = true;
            drive = false;
        }

        if (driver.a) {
            stowHardStops = false;
            alignRight = false;
            alignLeft  = false;
            drive = true;
        }

        /* Operator Commands */
        Oa = operator.a;
        Ob = operator.b;
        Ou = operator.dpad_up;
        Od = operator.dpad_down;
        Ol = operator.dpad_left;
        Or = operator.dpad_right;

        lx = operator.left_stick_x;
        ly = operator.left_stick_y;
        ry = operator.right_stick_y;
        rx = operator.right_stick_x;

        // LiftModule.java
        lift = -operator.left_stick_y;

        if (operator.right_bumper) {
            clamp = true;
            approach = false;
            clampStowed = false;
        }

        if (operator.left_bumper) {
            clamp = false;
            approach = true;
            clampStowed = false;
        }

        if (operator.left_trigger > 0.5) {
            clamp = false;
            approach = false;
            clampStowed = true;
        }

        // Lift.java
        if (liftWatch.millis() >= 200) {
            stowLift = operator.x;
            extendLift = operator.b;
            incrementLift = operator.y;
            decrementLift = operator.a;
            liftWatch.reset();
            liftWatch.start();
        } else {
            stowLift = false;
            extendLift = false;
            incrementLift = false;
            decrementLift = false;
        }
        microAdjustLift = Range.scale(-operator.left_stick_y,-1,1,-0.05,0.05);
    }

}
