package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Settings;
import java.lang.Math;

/**
 * This is the drive state for a holonomic drive train.
 * Angles are measured relative to the front of the robot, with unit circle convention
 */

public class DriveState extends State {

    private double driveSpeed_x;
    private double driveSpeed_y;
    private double maxSpeed;
    private double distance;
    private int position;
    private int position_x;
    private int position_y;
    private int flTargetPosition;
    private int frTargetPosition;
    private int blTargetPosition;
    private int brTargetPosition;
    private DcMotor m0;
    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private final double wheelCircumference = (1.97 * 2) * Math.PI;
    private int ticksPerTurn = 1120;
    private boolean m0Reached = false;
    private boolean m1Reached = false;
    private boolean m2Reached = false;
    private boolean m3Reached = false;
    private int threshold = 75;
    private PIDController pidDrive_x;
    private PIDController pidDrive_y;
    private int direction;
    private Telemetry telemetry;
    private double realSpeed;

    /**
     public DriveState(double target, double speed, HardwareMap hardwareMap, Telemetry telemetry) {
     super(hardwareMap);
     this.telemetry = telemetry;
     distance = target;
     driveSpeed = speed;
     }
     */

    //new method for beta PID-drive
    public DriveState(double distance, double maxSpeed, int direction, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.distance = distance;
        this.maxSpeed = maxSpeed;
        this.direction = (direction + 45) % 360;  // +45 for the old auton files
        this.telemetry = telemetry;

        // initialize the motors with the hardware map

        m0 = hardwareMap.get(DcMotor.class, "m0");
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        m0.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.REVERSE);

        m1.setDirection(DcMotor.Direction.FORWARD);
        m2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        this.running = true;

        m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /**
         int currentPosition = (int)((fl.getCurrentPosition() +
         fr.getCurrentPosition() +
         bl.getCurrentPosition() +
         br.getCurrentPosition()) / 4.0);
         */
        int currentPosition = (m0.getCurrentPosition()
                + m1.getCurrentPosition()
                + m2.getCurrentPosition()
                + m3.getCurrentPosition()) / 4; // in case the encoders are not exactly at 0

        // position = currentPosition + TickService.inchesToTicks(distance);

        //int flTargetPosition = getFlTargetPosition(); //need fl motor position for PID calculations

        // calculate the separate positions (for x,y):

        // might need to reverse it, btw (meaning that we need -x for the position, based on the teleop)

        double distance_x = distance * Math.cos(Math.toRadians(direction));
        double distance_y = distance * Math.sin(Math.toRadians(direction));

        position_x = currentPosition + TickService.inchesToTicks(distance_x);
        position_y = currentPosition + TickService.inchesToTicks(distance_y);

        m0.setTargetPosition(position_x);
        m2.setTargetPosition(position_x);

        m1.setTargetPosition(position_y);
        m3.setTargetPosition(position_y);

        // setTargets(); // set target positions for each motor

        m0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // separate PID control for x,y

        // pidDrive_x = new PIDController(m0,1.7, 0.001, 0.6, hardwareMap, flTargetPosition, maxSpeed);
        // pidDrive_y = new PIDController(m1,1.7, 0.001, 0.6, hardwareMap, flTargetPosition, maxSpeed);

        // driveSpeed_x = pidDrive_x.PIDControl();
        // driveSpeed_y = pidDrive_y.PIDControl();

        driveSpeed_x = 0.7; // this is only for initial testing
        driveSpeed_y = 0.7;

        drive(driveSpeed_x,driveSpeed_y);
    }

    @Override
    public void update() {
        /**
         flReached = Math.abs(fl.getCurrentPosition()) >= Math.abs(position) - threshold;
         //frReached = Math.abs(fr.getCurrentPosition()) >= Math.abs(position) - threshold;
         frReached = true;
         blReached = Math.abs(bl.getCurrentPosition()) >= Math.abs(position) - threshold;
         brReached = Math.abs(br.getCurrentPosition()) >= Math.abs(position) - threshold;
         */

        m0Reached = Math.abs(m0.getCurrentPosition() - position_x) < threshold;
        m1Reached = Math.abs(m1.getCurrentPosition() - position_y) < threshold;
        m2Reached = Math.abs(m2.getCurrentPosition() - position_x) < threshold;
        m3Reached = Math.abs(m3.getCurrentPosition() - position_y) < threshold;

        // realSpeed = pidDrive.getActualSpeed();

        if (m0Reached && m1Reached && m2Reached && m3Reached) {
            this.stop();
            this.goToNextState();
        }
        else {
            // driveSpeed_x = pidDrive_x.PIDControl();
            // driveSpeed_y = pidDrive_y.PIDControl();
            drive(driveSpeed_x,driveSpeed_y);
        }

        telemetry.addLine("m1 Diff: " + Math.abs(m1.getCurrentPosition() - position_y));
        telemetry.addLine("m1 Pos: " + m1.getCurrentPosition());
        telemetry.addLine("m1 Power: " + m1.getPower());
        telemetry.addLine("m0 Diff: " + Math.abs(m0.getCurrentPosition() - position_x));
        telemetry.addLine("m2 Diff: " + Math.abs(m2.getCurrentPosition() - position_x));
        telemetry.addLine("m3 Diff: " + Math.abs(m3.getCurrentPosition() - position_y));
        // telemetry.addLine("actualSpeed : " + realSpeed);
    }

    @Override
    public void stop() {
        drive(0,0);
        this.running = false;
    }

    @Override
    public String toString() {
        return "DriveState: Power X = " + driveSpeed_x + ", Distance =" + distance;
    }

    //obsolete once all four enocders are used, can simply set a constant power
    //the target encoder value is what matters
    // we need to adjust based on x and y, rip
    public void drive(double driveSpeed_x, double driveSpeed_y) {
        m0.setPower(driveSpeed_x);
        m1.setPower(driveSpeed_y);
        m2.setPower(driveSpeed_x);
        m3.setPower(driveSpeed_y);
    }

    //target position changes based on direction of motion
    private void setTargets() {

        /**

        double m0_power = -gamepad1.left_stick_x; // left or right
        double m2_power = -gamepad1.left_stick_x; // left or right

        double m3_power = gamepad1.left_stick_y; // up or down
        double m1_power = gamepad1.left_stick_y; // up or down

        m0.setPower(m0_power);
        m2.setPower(m2_power);

        m3.setPower(m3_power);
        m1.setPower(m1_power);

        // for setting the target position, i handled the direction so like
        // i think we can just set the target positions in the setup method

        switch (direction) {
            case "front":
                flTargetPosition = position;
                frTargetPosition = position;
                blTargetPosition = position;
                brTargetPosition = position;
                break;
            case "back":
                flTargetPosition = -position;
                frTargetPosition = -position;
                blTargetPosition = -position;
                brTargetPosition = -position;
                break;
            case "left":
                flTargetPosition = -position;
                frTargetPosition = position;
                blTargetPosition = position;
                brTargetPosition = -position;
                break;
            case "right":
                flTargetPosition = position;
                frTargetPosition = -position;
                blTargetPosition = -position;
                brTargetPosition = position;
                break;
        }

        fl.setTargetPosition(flTargetPosition);
        fr.setTargetPosition(frTargetPosition);
        bl.setTargetPosition(blTargetPosition);
        br.setTargetPosition(brTargetPosition);
         *
         */
    }
}