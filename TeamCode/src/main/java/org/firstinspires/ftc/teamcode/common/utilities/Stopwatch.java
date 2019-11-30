package org.firstinspires.ftc.teamcode.common.utilities;

public class Stopwatch {
    private double currentTime;
    private double timestamp;
    private double delta;
    private enum STATE {STOPPED, RUNNING}
    private STATE state;

    public Stopwatch() {
        currentTime = System.currentTimeMillis();
        timestamp = currentTime;
        delta = 0;
        state = STATE.STOPPED;
    }

    public int start() {
        if (state == STATE.STOPPED) {
            currentTime = System.currentTimeMillis();
            timestamp = currentTime;
            delta = 0;
            state = STATE.RUNNING;
            return 0;
        }
        return -1;
    }

    public int stop() {
        if (state == STATE.RUNNING) {
            currentTime = System.currentTimeMillis();
            delta = currentTime - timestamp;
            timestamp = currentTime;
            state = STATE.STOPPED;
            return 0;
        }
        return -1;
    }

    public int reset() {
        int status = stop();
        currentTime = System.currentTimeMillis();
        timestamp = currentTime;
        delta = 0;
        return status;
    }

    public double millis() {
        if (state == STATE.RUNNING)
            return System.currentTimeMillis() - timestamp;
        return delta;
    }

    public double seconds() {
        return millis()/1000;
    }

    public boolean isRunning() {
        return state == STATE.RUNNING;
    }

    /*
     * TODO Implement "laps"
     *      Store laps in an ascending sorted list while the stopwatch is still running
     *      Clear the laps once stopwatch is stopped
     */
}