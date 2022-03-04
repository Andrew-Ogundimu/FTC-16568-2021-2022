package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.IMU;

/**
 * This is the autonomous state machine, where we make and run the states.
 * This class is for depositing the pre-loaded element into the shipping container
 */

@Autonomous(name = "BlueElement")
public class BlueElement extends OpMode {

    // INSTANCE VARIABLES
    /**
     * Version of the op-mode file.
     */
    private final double VERSION = 1.0;

    /**
     * The first state to be run.
     */
    private State headerState;
    private State arm_state;

    // private IMU imu;

    // METHODS

    /**
     * Sets up all relevant things for the op-mode.
     */
    @Override
    public void init() {

        State[] defaultStateSequence = {
                new GrabState(-1.0,1, hardwareMap, telemetry),  // grab element
                new SensorState(hardwareMap,telemetry), // detect capstone position
                new DriveState(5, 0.7, 0, hardwareMap,telemetry), // move away from the wall
                new TurnArcState(90,hardwareMap,telemetry), // turn so that the arm faces forwards
                new DriveState(24,0.7, 0, hardwareMap, telemetry), // move sideways (away from elements)
                new DriveState(17,0.7, -90, hardwareMap, telemetry), // move forwards (towards the shipping hub)
                new ArmState(290,0, hardwareMap,telemetry), // raise the arm to the specified level
                new DriveState(7,0.7,-90,hardwareMap,telemetry), // move forwards (towards the shipping hub)
                new GrabState(1.0,1, hardwareMap, telemetry), // release element
        };
        arm_state = defaultStateSequence[6];
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
