package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.IMU;

/**
 * This is the autonomous state machine, where we make and run the states.
 */

@Autonomous(name = "BlueWheel_FarPark")
public class BlueWheel_FarPark extends OpMode {

    // INSTANCE VARIABLES
    /**
     * Version of the op-mode file.
     */
    private final double VERSION = 1.0;

    /**
     * The first state to be run.
     */
    private State headerState;

    // private IMU imu;

    // METHODS

    /**
     * Sets up all relevant things for the op-mode.
     */
    @Override
    public void init() {

        State[] defaultStateSequence = {
                new DriveState(5, 0.1, -90, hardwareMap, telemetry),
                new DriveState(11.5, 0.1, 180, hardwareMap, telemetry),
                new WheelState(1, 5, hardwareMap, telemetry),
                new DriveState(11.5, 0.1, 0, hardwareMap, telemetry),
                new DriveState(5, 0.1, 90, hardwareMap, telemetry),
                new DriveState(98-11.5, 0.1, 0, hardwareMap, telemetry),
        };

        headerState = StateBuilder.buildStates(defaultStateSequence);
    }

    /**
     * Runs all things related to starting the op-mode.
     */
    @Override
    public void start() {
        this.headerState.start();
    }

    @Override
    public void loop() {
        State currentState = headerState.getCurrentState();
        boolean running = currentState != null;

        // Update State
        if (running) {
            currentState.update();
        }

        String status = running ? "RUNNING" : "COMPLETED";
        String currentStateString = running ? currentState.toString() : "None";

        // State telemetry
        telemetry.addLine("CurrentState: " + currentStateString);
        telemetry.addLine("Status: " + status);
        // telemetry.addLine("Orientation: " + this.imu.getOrientation());

        // Version telemetry.
        telemetry.addLine("Version: " + this.VERSION);
    }

    @Override
    public void stop() {
        State currentState = headerState.getCurrentState();

        if (currentState != null) {
            currentState.stop();
        }

        // this.imu.close();
    }
}
