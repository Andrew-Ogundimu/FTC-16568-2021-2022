package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.IMU;

/**
 * This is the autonomous state machine, where we make and run the states.
 */

@Autonomous(name = "RedNearPark")
public class RedNearPark extends OpMode {

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
                new DriveState(36, 0.9, 45, hardwareMap, telemetry),
        };

        // this.imu = IMU.getInstance(IMU.class, hardwareMap);

        headerState = StateBuilder.buildStates(defaultStateSequence);
    }

    /**
     * Runs all things related to starting the op-mode.
     */
    @Override
    public void start() {
        // this.imu.setDefaultOrientation();
        this.headerState.start();
    }

    @Override
    public void loop() {
        State currentState = headerState.getCurrentState();
        boolean running = currentState != null;

        String status = running ? "RUNNING" : "COMPLETED";
        String currentStateString = running ? currentState.toString() : "None";

        // State telemetry
        telemetry.addLine("CurrentState: " + currentStateString);
        telemetry.addLine("Status: " + status);
        // telemetry.addLine("Orientation: " + this.imu.getOrientation());

        // Update State
        if (running) {
            currentState.update();
        }

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
