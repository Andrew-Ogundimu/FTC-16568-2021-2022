package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auton.TickService;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.IMU;

/**
 * This is the turn state, using encoders
 * Angles are measured relative to the front of the robot, with unit circle convention
 */
public class TurnArcState extends State {

    private DcMotor m0;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;

    private final IMU imu;

    private double gyroTarget;

    private final double gyroRange = 5; //range of 5 degrees
    private double gyroError = 0;
    private double gyroPrevError = 0;
    private double gyroDiffError = 0;
    private double gyroSumError = 0;

    private boolean firstTurnIteration = true;

    private ElapsedTime runtime = new ElapsedTime();
    private int timeout = 5;
    private Telemetry telemetry;

    private double turn_distance;
    final double turn_radius = 6.0;

    private int turn_target = 0;

    private boolean m0Reached = false;
    private boolean m1Reached = false;
    private boolean m2Reached = false;
    private boolean m3Reached = false;

    private int threshold = 75;

    /**
     * Default state constructor
     * @param hardwareMap
     */
    public TurnArcState(double gyroTarget, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);

        this.imu = IMU.getInstance(IMU.class, hardwareMap);

        this.gyroTarget = gyroTarget; //target angle
        this.telemetry = telemetry;

        m0 = hardwareMap.get(DcMotor.class, "m0");
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
    }

    @Override
    public void start() {
        this.running = true;
        runtime.reset();

        turn_target = (int) TickService.inchesToTicks(turn_radius * Math.toRadians(gyroTarget));

        m0.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.REVERSE);

        m1.setDirection(DcMotor.Direction.FORWARD);
        m2.setDirection(DcMotor.Direction.REVERSE);

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m0.setTargetPosition(turn_target);
        m2.setTargetPosition(-turn_target);

        m1.setTargetPosition(turn_target);
        m3.setTargetPosition(-turn_target);

        m0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(.3); // start at half power
    }

    @Override
    public void update() {
        m0Reached = Math.abs(m0.getCurrentPosition() - turn_target) < threshold;
        m1Reached = Math.abs(m1.getCurrentPosition() - turn_target) < threshold;
        m2Reached = Math.abs(m2.getCurrentPosition() + turn_target) < threshold;
        m3Reached = Math.abs(m3.getCurrentPosition() + turn_target) < threshold;

        // realSpeed = pidDrive.getActualSpeed();

        if (m0Reached && m1Reached && m2Reached && m3Reached) {
            this.stop();
            this.goToNextState();
        }

        telemetry.addLine("Current Angle: " + this.imu.getOrientation());
    }

    @Override
    public void stop() {
        turn(0);
        this.running = false;
    }

    @Override
    public String toString() {
        return "Turn state";
    }

    // in case we need to scale down the power later
    private void gyroCorrect(double gyroTarget) {

        final double minSpeed = 0.7;
        final double addSpeed = 0.2;

        double gyroActual = imu.getOrientation(); // current heading or angle
        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; // in case it is negative
        if (delta > 180.0) {
            delta -= 360.0; // delta becomes between -180 and 180
            //because the range is from 0-> 180 and -180-> 0 instead of 0-> 360
        }
        if (Math.abs(delta) > gyroRange) {
            double gyroMod = delta / 45.0; // if delta is less than 45 and bigger than -45, this will make a scale from
            // -1 to 1
            if (Math.abs(gyroMod) > 1.0) {
                gyroMod = Math.signum(gyroMod); //makes gyroMod -1 or 1 if error is more than 45
                // or less than -45 degrees
            }
            //if the error is more than 180, then the power is positive, and it turns to the left
            //if the error is less than 180, the power in the turn in negative, and it turns to the
            //right
            //if the error is larger, faster speed
            this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
        }
        else {
            turn(0.0);
        }
     }

     // using PID instead
    private void gyroTurn(double gyroTarget) { //turn function utilizing the imu, and for 90 degree turns

        double gyroActual = imu.getOrientation();

        final double minSpeed = 0.7;
        final double addSpeed = 0.2;

        double delta = (gyroTarget - gyroActual + 360.0) % 360.0; // in case it is negative

        if (delta > 180.0) { //normalize function
            delta -= 360.0; // delta becomes between -180 and 180
            //because the range is from 0-> 180 and -180-> 0 instead of 0-> 360
        }
        if (Math.abs(delta) > gyroRange) { //not close enough to end position
            //double gyroMod = delta / 45.0; //if delta is less than 45 and bigger than -45, this will make a scale from
            // -1 to 1
            if (firstTurnIteration) {
                double gyroMod = delta / 45.0; // if delta is less than 45 and bigger than -45, this will make a scale from
                // -1 to 1

                if (Math.abs(gyroMod) > 1.0) {
                    gyroMod = Math.signum(gyroMod); //makes gyroMod -1 or 1 if error is more than 45
                    // or less than -45 degrees
                }

                //if the error is more than 180, then the power is positive, and it turns to the left
                //if the error is less than 180, the power in the turn in negative, and it turns to the
                //right
                //if the error is larger, faster speed

                //go full speed at first
                this.turn(minSpeed * Math.signum(gyroMod) + addSpeed * gyroMod);
                firstTurnIteration = false;
            }
            else { //PID control
                gyroError = delta / Math.abs(gyroTarget); //for a 90 degree turn, assuming 90 is the target position
                //and so that we do not change the sign of error

                //makes a scale for delta from 0 to 1, the closer it gets the smaller error becomes
                gyroDiffError = gyroError - gyroPrevError;

                gyroSumError += gyroError;

                double turnSpeed = gyroError * 1.3 + gyroDiffError * 0.8 + gyroSumError * 0.0001;
                this.turn(turnSpeed);

                gyroPrevError = gyroError;
            }
            // prevHeading = (float)getCurrentHeading(); //set the last heading
        }
        else {
            turn(0.0);
        }
    }

    private void turn(double power) {
        // no direction change necessary
        m0.setPower(power);
        m1.setPower(power);
        m2.setPower(-power);
        m3.setPower(-power);
    }
}
