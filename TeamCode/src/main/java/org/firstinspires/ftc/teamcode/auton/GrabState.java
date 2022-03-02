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
    private CRServo grab = null;

    public GrabState(double power, int time, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.power = power;
        this.time = time;
        this.telemetry = telemetry;

        // initialize the servo with the hardware map
        grab = hardwareMap.get(CRServo.class,"grab");
    }

    @Override
    public void start() {
        this.running = true;
        initial_time = System.currentTimeMillis();

        grab.setPower(power);
    }

    @Override
    public void update() {
        long elapsed_milliseconds = System.currentTimeMillis() - initial_time;
        long elapsed_seconds = (elapsed_milliseconds / 1000) % 60;

        if (elapsed_seconds >= time) { // if enough seconds have passed
            this.stop();
            this.goToNextState();
        }

        telemetry.addLine("Servo power: " + grab.getPower());
    }

    @Override
    public void stop() {
        this.running = false;
    }

    @Override
    public String toString() {
        return "Servo Position = " + grab.getPower();
    }
}