package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is the state for controlling the grabber mechanism
 */

public class GrabState extends State {

    private double power;
    private Telemetry telemetry;
    private int time;
    private long initial_time;
    private Servo grab = null;
    private double position;

    public GrabState(double position, int time, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.position = position;
        this.time = time;
        this.telemetry = telemetry;

        // initialize the servo with the hardware map
        grab = hardwareMap.get(Servo.class,"grab");
    }

    @Override
    public void start() {
        this.running = true;
        initial_time = System.currentTimeMillis();

        grab.setPosition(position);
    }

    @Override
    public void update() {
        long elapsed_milliseconds = System.currentTimeMillis() - initial_time;
        long elapsed_seconds = (elapsed_milliseconds / 1000) % 60;

        if (elapsed_seconds >= time) { // if enough seconds have passed
            this.stop();
            this.goToNextState();
        }

        telemetry.addLine("Servo power: " + grab.getPosition());
    }

    @Override
    public void stop() {
        this.running = false;
    }

    @Override
    public String toString() {
        return "Servo Position = " + grab.getPosition();
    }
}