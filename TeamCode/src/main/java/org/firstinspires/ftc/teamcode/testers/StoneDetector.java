package org.firstinspires.ftc.teamcode.testers;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;

@TeleOp(name = "TestOp: Stone Detector")
public class StoneDetector extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, UniversalConstants.webcamName);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
        parameters.cameraName = webcamName;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.GRAYSCALE);
        vuforia.setFrameQueueCapacity(1);

        Bitmap bitmap = vuforia.convertFrameToBitmap(vuforia.getFrameQueue().take());
        telemetry.addData("Height", bitmap.getHeight());
        telemetry.addData("Width", bitmap.getWidth());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }


    public int rgbToGrey(int color) {
        int R = (color >> 16) & 0xff;
        int G = (color >>  8) & 0xff;
        int B = (color      ) & 0xff;
//        (0.3 * R) + (0.59 * G) + (0.11 * B)
        int result = (int) Math.round(0.3*R + 0.59*G + 0.11*B);
        return result;
    }
}
