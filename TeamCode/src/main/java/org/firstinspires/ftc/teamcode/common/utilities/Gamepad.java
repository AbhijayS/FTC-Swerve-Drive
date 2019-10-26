package org.firstinspires.ftc.teamcode.common.utilities;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// TODO: Port all gamepad commands here
public class Gamepad {
    private com.qualcomm.robotcore.hardware.Gamepad driver;
    private com.qualcomm.robotcore.hardware.Gamepad operator;

    private LinearOpMode linearOpMode;
    public boolean _0, _45, _90, _135, _180, _225, _270, _315, heading, Oa, Ob, Ou, Ol, Or, Od;

    public double x, y, z, lx, ly, rx, ry;

    private final int leeway = 10;
    private int counter;

    public Gamepad(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.driver = linearOpMode.gamepad1;
        this.operator = linearOpMode.gamepad2;
        this.counter = 0;
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




        linearOpMode.telemetry.addData("0", _0);
        linearOpMode.telemetry.addData("90", _90);
        linearOpMode.telemetry.addData("180", _180);
        linearOpMode.telemetry.addData("270", _270);
    }

}
