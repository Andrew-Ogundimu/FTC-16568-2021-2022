package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.hardware.IMU;

public class TurnState extends State {

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

    /**
     * Default state constructor
     * @param hardwareMap
     */
    public TurnState(double gyroTarget, HardwareMap hardwareMap, Telemetry telemetry) {
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

        this.imu.initialize();
        this.imu.setDefaultOrientation();

        m0.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.REVERSE);

        m1.setDirection(DcMotor.Direction.FORWARD);
        m2.setDirection(DcMotor.Direction.REVERSE);

        //must set the runMode to run without encoder in order for it to run
        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turn(.5); // start at half power
    }

    @Override
    public void update() {
        if (Math.abs(gyroTarget - imu.getOrientation()) < gyroRange || runtime.seconds() > timeout) { //reached target or too much elapsed time
            this.stop();
            this.goToNextState();
        }
        else {
            firstTurnIteration = false;
        }
        //else { we don't need this because it's just going to turn at half speed until it reaches the target
            //gyroCorrect(gyroTarget); //re-run method to adjust speed
        //}
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
        m2.setPower(power);
        m3.setPower(power);
    }
}
