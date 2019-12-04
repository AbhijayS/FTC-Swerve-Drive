package org.firstinspires.ftc.teamcode.testers;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.common.UniversalConstants;

@TeleOp(name = "TestOp: Stone Detector")
public class StoneDetector extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = UniversalConstants.vuforiaLicenceKey;
        parameters.cameraName = webcamName;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        vuforia.enableConvertFrameToFormat(PIXEL_FORMAT.GRAYSCALE);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565,true);
        vuforia.setFrameQueueCapacity(2);

        telemetry.addLine("Ready to go!");
        telemetry.update();
        waitForStart();

        // get image from vuforia
        Frame frame = vuforia.getFrameQueue().take();
        telemetry.addData("Vu Height", frame.getImage(1).getHeight());
        telemetry.addData("Vu Width", frame.getImage(1).getWidth());
        telemetry.addData("Vu Format", frame.getImage(1).getFormat() == PIXEL_FORMAT.RGB565);

        // build an android bitmap
        Image vu_img = frame.getImage(1);
        Bitmap bitmap = Bitmap.createBitmap(vu_img.getWidth(), vu_img.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(vu_img.getPixels());

        telemetry.addLine();
        telemetry.addData("Bit Height", bitmap.getHeight());
        telemetry.addData("Bit Width", bitmap.getWidth());
        telemetry.addData("Pixel", bitmap.getPixel(0,0));
        telemetry.addData("Gray", rgbToGray(bitmap.getPixel(0,0)));

        telemetry.update();
        while (opModeIsActive()) {
            idle();
        }
    }


    public int rgbToGray(int color) {
        int R = (color >> 16) & 0xff;
        int G = (color >>  8) & 0xff;
        int B = (color      ) & 0xff;
        int gray = (int) Math.round(0.3*R + 0.59*G + 0.11*B);
        return gray;
    }
}
