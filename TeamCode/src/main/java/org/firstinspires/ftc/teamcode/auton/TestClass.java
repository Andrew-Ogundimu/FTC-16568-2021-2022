package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.hardware.IMU;

/**
 * This is the autonomous state machine, where we make and run the states.
 * This class is for depositing the pre-loaded element into the shipping container
 */

@Autonomous(name = "TestClass")
public class TestClass extends OpMode {

    // INSTANCE VARIABLES
    /**
     * Version of the op-mode file.
     */
    private final double VERSION = 1.0;

    /**
     * The first state to be run.
     */
    private State headerState;

    private State sensorState;

    // private IMU imu;

    // METHODS

    /**
     * Sets up all relevant things for the op-mode.
     */
    @Override
    public void init() {

        State[] defaultStateSequence = {
                new SleepState(3,hardwareMap,telemetry),
                new SensorState(hardwareMap,telemetry),
                new ArmState(255,0, hardwareMap,telemetry)
                //new ArmState(1, 60, hardwareMap,telemetry)
        };

        /*
        State[] defaultStateSequence = {
                new SleepState(3,hardwareMap,telemetry),
                new DriveState(10,0.7,0-90,hardwareMap,telemetry)
                //new ArmState(70, 60, hardwareMap,telemetry)
        };
        */

        for (State state: defaultStateSequence) {
            state.init();
        }

        sensorState = defaultStateSequence[0];

        /**

        State[] defaultStateSequence = {
                new DriveState(5, 0.7, 90, hardwareMap,telemetry),
                new NewArmState(290,0,hardwareMap,telemetry)
        };
         */

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
        // String currentStateString = running ? currentState.toString() : "None";
        String currentStateString = sensorState.toString();

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

