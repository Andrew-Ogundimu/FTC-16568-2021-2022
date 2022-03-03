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

    private DcMotor motor;

    private int targetPosition;

    public PIDController(DcMotor motor, double Kp, double Ki, double Kd, HardwareMap hardwareMap, int position, double maxSpeed) {
        this.motor = motor; // specify the motor that we want to control (either x or y motor)
        kp = Kp;
        ki = Ki;
        kd = Kd;
        targetPosition = position;
        maxOutput = maxSpeed;
    }

    public double PIDControl() {
        prev_error = error;

        double currentPosition = motor.getCurrentPosition();

        error = Math.abs(currentPosition - targetPosition); // find the difference between the current and target position

        error /= Math.abs(targetPosition);

        diff_error = error - prev_error; // change in error

        sum_error += error; // add error to total error

        double driveSpeed = kp * error + ki * sum_error + kd * diff_error; // calculate the new power

        driveSpeed = constrain(driveSpeed); // constrain the power to a certain range

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
}
