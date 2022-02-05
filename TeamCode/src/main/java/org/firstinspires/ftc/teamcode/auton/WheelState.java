package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is the wheel state for a continuous servo.
 */

public class WheelState extends State {

    private double power;
    private Telemetry telemetry;
    private int time;
    private long initial_time;
    private DcMotor wheel;

    public WheelState(double power, int time, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.power = power;
        this.time = time;
        this.telemetry = telemetry;

        // initialize the servo with the hardware map
        wheel = hardwareMap.get(DcMotor.class,"wheel"); // we'll put this in a separate state later
    }

    @Override
    public void start() {
        this.running = true;
        initial_time = System.currentTimeMillis();

        wheel.setPower(power);
    }

    @Override
    public void update() {
        long elapsed_milliseconds = System.currentTimeMillis() - initial_time;
        long elapsed_seconds = (elapsed_milliseconds / 1000) % 60;

        if (elapsed_seconds >= time) { // if enough seconds have passed
            this.stop();
            this.goToNextState();
        }

        telemetry.addLine("Servo power: " + wheel.getPower());
    }

    @Override
    public void stop() {
        wheel.setPower(0);
        this.running = false;
    }

    @Override
    public String toString() {
        return "Wheel Power = " + wheel.getPower();
    }
}