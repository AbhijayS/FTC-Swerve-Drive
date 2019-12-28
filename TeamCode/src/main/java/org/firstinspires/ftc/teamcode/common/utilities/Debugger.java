package org.firstinspires.ftc.teamcode.common.utilities;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.common.UniversalConstants.ROBOT_STATUS;
import static org.firstinspires.ftc.teamcode.common.UniversalConstants.Status;

public class Debugger {
    private final String fileExtension = ".csv";

    private ArrayList<Object> buffer;
    private ArrayList<String> categories;
    private int size;
    private Context context;
    private FileOutputStream fOut;
    private OutputStreamWriter outputStreamWriter;
    private CSVFormat csvFormat;
    private CSVPrinter csvPrinter;
    private String fileName;
    private Stopwatch stopwatch;
    private boolean initialized;
    private Telemetry telemetry;

    public enum Marker {
        VEL("Velocity"),
        ACCEL("Acceleration"),
        RX("Robot X"),
        RY("Robot Y"),
        PX("Tracking Pose X"),
        PY("Tracking Pose Y"),
        HEADING("Heading"),
        PATH("Path");

        private final String name;

        Marker(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }

        public static List<String> getDebuggingMarkers() {
            ArrayList<String> result = new ArrayList<>();
            for (Marker d : Marker.values()) {
                result.add(d.toString());
            }
            return result;
        }

    }

    public Debugger(Context context, LinearOpMode linearOpMode, ArrayList<String> categories) {
        this.context = context;
        this.telemetry = linearOpMode.telemetry;
        this.buffer = new ArrayList<>();
        this.categories = new ArrayList<>();
        this.buffer.add(new Object());
        this.categories.add("Timestamp");

        for (String category : categories) {

            // Make sure Timestamp is unique
            if (category.equals("Timestamp"))
                throw new RuntimeException("Cannot have a category named 'Timestamp'!");

            // ignore duplicate entries
            if (this.categories.contains(category))
                continue;

            this.categories.add(category);
            this.buffer.add("");
        }

        this.size = categories.size();
        this.stopwatch = new Stopwatch();
        this.initialized = false;
    }

    public void addData(String category, Object data) {
        int index = categories.indexOf(category);
        if (ROBOT_STATUS != Status.RELEASE)
            telemetry.addData(category, data.toString());

        if (index == -1)
            return;

        buffer.set(index, data.toString());
    }

    public void log() {
        if (initialized) {
            try {
                stopwatch.start();
                buffer.set(0, Double.toString(stopwatch.millis()));
                csvPrinter.printRecord(buffer);
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            throw new RuntimeException("Debugger is not initialized properly!");
        }

        telemetry.update();
    }

    public void initialize(String fileName) {
        if (fileName.length() < 1)
            throw new RuntimeException("File name is too short!");
        this.fileName = fileName.concat(fileExtension);

        try {
            fOut = context.openFileOutput(this.fileName, Context.MODE_PRIVATE);
            outputStreamWriter = new OutputStreamWriter(fOut);
            csvFormat = CSVFormat.EXCEL.withFirstRecordAsHeader();
            csvPrinter = new CSVPrinter(outputStreamWriter, csvFormat);
        } catch (IOException ioe) {
            ioe.printStackTrace();
        }

        if (size >= 2) {
            try {
                csvPrinter.printRecord(categories);
            } catch (IOException e) {
                e.printStackTrace();
            }
            initialized = true;
        }

        if (initialized)
            telemetry.addLine("Debugger Initialized!");
    }

    public void stopLogging() {
        if (initialized) {
            try {
                csvPrinter.flush();
                csvPrinter.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } else {
            throw new RuntimeException("Debugger is not initialized properly!");
        }

        stopwatch.stop();
    }
}