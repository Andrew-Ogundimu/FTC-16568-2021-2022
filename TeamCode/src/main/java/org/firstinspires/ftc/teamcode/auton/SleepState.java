package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is to stop the robot for a certain amount of time
 */

public class SleepState extends State {

    private Telemetry telemetry;
    private int time;
    private long initial_time;

    public SleepState(int time, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.time = time;
        this.telemetry = telemetry;
    }

    @Override
    public void start() {
        this.running = true;
        initial_time = System.currentTimeMillis();
    }

    @Override
    public void update() {
        long elapsed_milliseconds = System.currentTimeMillis() - initial_time;
        long elapsed_seconds = (elapsed_milliseconds / 1000) % 60;

        if (elapsed_seconds >= time) { // if enough seconds have passed
            this.stop();
            this.goToNextState();
        }

        telemetry.addLine("Seconds Passed: " + elapsed_seconds);
    }

    @Override
    public void stop() {
        this.running = false;
    }

    @Override
    public String toString() {
        return "Total Time: " + time;
    }
}