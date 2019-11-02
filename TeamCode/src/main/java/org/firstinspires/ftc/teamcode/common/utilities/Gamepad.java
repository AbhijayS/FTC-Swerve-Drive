package org.firstinspires.ftc.teamcode.common.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// TODO: Port all gamepad commands here
public class Gamepad {
    private com.qualcomm.robotcore.hardware.Gamepad driver;
    private com.qualcomm.robotcore.hardware.Gamepad operator;

    private LinearOpMode linearOpMode;
    // driver controls
    public boolean _0, _45, _90, _135, _180, _225, _270, _315, heading, Oa, Ob, Ou, Ol, Or, Od, slowmo;

    // operator controls
    public boolean clamp;
    public boolean unclamp;
    public double lift;

    public double x, y, z, lx, ly, rx, ry;

    public Gamepad(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.driver = linearOpMode.gamepad1;
        this.operator = linearOpMode.gamepad2;
        update();
    }

    public void update() {
        _0 = driver.dpad_right;
        _90 = driver.dpad_up;
        _180 = driver.dpad_left;
        _270 = driver.dpad_down;
        Oa = operator.a;
        Ob = operator.b;
        Ou = operator.dpad_up;
        Od = operator.dpad_down;
        Ol = operator.dpad_left;
        Or = operator.dpad_right;
        slowmo = driver.left_bumper;

        heading = (_0 || _90 || _180 || _270) && !(_0 && _90 && _180 && _270);

        _45 = _0 && _90;
        _135 = _90 && _180;
        _225 = _180 && _270;
        _315 = _270 && _0;

        x = driver.left_stick_x;
        y = -driver.left_stick_y;
        z = driver.right_stick_x;
        lx = operator.left_stick_x;
        ly = operator.left_stick_y;
        ry = operator.right_stick_y;
        rx = operator.right_stick_x;

        lift = -operator.left_stick_y;
        clamp = operator.right_bumper;
        unclamp = operator.left_bumper;
    }

}
