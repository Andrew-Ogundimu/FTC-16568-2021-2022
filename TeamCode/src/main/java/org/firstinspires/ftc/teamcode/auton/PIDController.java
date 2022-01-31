package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Settings;

public class PIDController {

    private double kp;
    private double kd;
    private double ki;
    private double maxOutput;
    private double minOutput = 0.2;

    private double error = 0;
    private double prev_error;
    private double diff_error;
    private double sum_error;

    // private DcMotor m0;
    // private DcMotor m1;
    // private DcMotor m2;
    // private DcMotor m3;

    private DcMotor motor;

    private int targetPosition;

    private double actualSpeed;

    public PIDController(DcMotor motor, double Kp, double Ki, double Kd, HardwareMap hardwareMap, int position, double maxSpeed) {
        this.motor = motor; // specify the motor that we want to control (either x or y)
        kp = Kp;
        ki = Ki;
        kd = Kd;
        targetPosition = position;
        maxOutput = maxSpeed;

        //m0 = hardwareMap.get(DcMotor.class, "m0");
        //m1 = hardwareMap.get(DcMotor.class, "m1");
        //m2 = hardwareMap.get(DcMotor.class, "m2");
        //m3 = hardwareMap.get(DcMotor.class, "m3");

        //double elapsedTime = 1.0;
    }

    public double PIDControl() {
        prev_error = error;
        //obtain the error, distance from the target

        /**
         double averageCurrentPosition = (fl.getCurrentPosition() + fr.getCurrentPosition() + bl.getCurrentPosition()
         + br.getCurrentPosition()) / 4.0;
         */

        //for strafing, this method fails
        //we should thus take the position from one motor only
        /**
         double averageCurrentPosition = (fl.getCurrentPosition() + bl.getCurrentPosition()
         + br.getCurrentPosition()) / 3.0;
         error = Math.abs(averageCurrentPosition - targetPosition);
         */
        double currentPosition = motor.getCurrentPosition();
        error = Math.abs(currentPosition - targetPosition);

        //to make the error fit within the range of 0-1, divide it by the original targetPosition
        //i.e if the target position is 1600, error = 1600/1600 = 1
        //robot approaches target, error decreases

        // scales from 1 to 0
        error /= Math.abs(targetPosition); //halfway becomes error of 0.5
        //must divide by the magnitude of the targetPosition
        //otherwise error may be negative, which causes calculation issues

        //if targetPosition is negative (in the case of moving backward) then error is also negative

        //this diff error would thus be the current error (negative) minus the previous error, also negative
        //so this becomes more positive

        //so for the diff error,

        diff_error = error - prev_error; //change in error

        sum_error += error; //add error to total error

        // driveSpeed must be positive
        double driveSpeed = kp * error + ki * sum_error + kd * diff_error;

        // actualSpeed = driveSpeed;

        driveSpeed = constrain(driveSpeed); // make sure it's not outside of range

        return driveSpeed;
    }

    public double constrain(double speed){
        double driveSpeed = speed;

        if (driveSpeed > maxOutput) {
            driveSpeed = maxOutput;
        }
        else if (driveSpeed < minOutput) {
            driveSpeed = minOutput;
        }

        return driveSpeed;
    }
    public double getActualSpeed() {
        return actualSpeed;
    }
}
