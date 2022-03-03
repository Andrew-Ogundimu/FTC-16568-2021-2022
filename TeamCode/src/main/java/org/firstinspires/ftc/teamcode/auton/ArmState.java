package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Settings;

import java.lang.Math;
import java.util.Arrays;

/**
 * This is the arm state
 */

public class ArmState extends State {

    private int threshold = 20;
    private Telemetry telemetry;

    private DcMotor arm1;
    private Servo arm2;
    private double servo_range = 280;

    private ArmState.Arm total = new ArmState.Arm(250f,250f);

    final float[] start_pos = new float[]{105f,60f};

    final double initAngle = total.CalcServos(start_pos[0],start_pos[1])[0]*180;

    private float arm_speed = 3;
    private float[] angles = new float[]{0f,0f};

    private float[] targ_pos = start_pos.clone();
    private float[] last_targ = targ_pos.clone();

    final int tickRotation = 1680;

    private int pos_x;
    private int pos_y;

    private int target;

    //new method for beta PID-drive
    public ArmState(int pos_x, int pos_y, HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.pos_x = pos_x;
        this.pos_y = pos_y;
        this.telemetry = telemetry;

        arm1 = hardwareMap.get(DcMotor.class, "bottom_arm1");
        arm2 = hardwareMap.get(Servo.class,"top_arm1");

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {
        this.running = true;

        arm1.setPower(1.0f);

        float[] targ_pos = new float[]{pos_x,pos_y};
        float[] angles = total.CalcServos(targ_pos[0],targ_pos[1]);
        arm2.setPosition(clip(angles[1]*180/servo_range+(servo_range-180)/servo_range));

        target = (int)(((angles[0])*(tickRotation/2)-initAngle/180*tickRotation/2));

        arm1.setTargetPosition(target); //some function that implements angles[0]
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void update() {
        arm1.setPower(1.0f);

        float[] targ_pos = new float[]{pos_x,pos_y};
        angles = total.CalcServos(targ_pos[0],targ_pos[1]);
        arm2.setPosition(1.0-clip(angles[1]*180/servo_range+(servo_range-180)/servo_range));

        target = (int)(((angles[0])*(tickRotation/2)-initAngle/180*tickRotation/2));

        arm1.setTargetPosition(target); //some function that implements angles[0]
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(arm1.getCurrentPosition() - target) < threshold) { // reached target position
            this.stop();
            this.goToNextState();
        }

        last_targ = targ_pos.clone();

        telemetry.addLine("Target Position: " + this.pos_x+" "+this.pos_y);
        telemetry.addData("Angles",Float.toString(angles[0]*180.0f)+" "+Float.toString(angles[1]*180.0f));
        telemetry.update();
    }

    @Override
    public void stop() {
        this.running = false;
    }

    @Override
    public String toString() {
        telemetry.addData("Angles",Float.toString(angles[0]*180.0f)+" "+Float.toString(angles[1]*180.0f));
        telemetry.update();
        return ("Target Position: " + this.pos_x+" "+this.pos_y);
    }

    public void setHeight(int height) {
        this.pos_y = height;
    }

    public double clip(double input) {
        if (0<=input && input<=1) {
            return input;
        } else if (input>1) {
            return 1;
        } else if (input < 0) {
            return 0;
        }
        return -1;
    }

    public class Arm {
        /*
        Initializes the Arm segment lengths in cm
         */
        private float segment1;
        private float segment2;
        public Arm(float length1,float length2) {
            segment1 = length1;
            segment2 = length2;
        }
        public float[] CalcServos(float dx, float dy) {
            float targ1 = (float)(Math.acos(
                    (segment2*segment2-segment2*segment2-dx*dx-dy*dy)/
                            (-2*segment1*Math.sqrt(dx*dx+dy*dy)))+Math.atan2(dy,dx));
            float targ2 = (float)Math.acos(
                    (dx*dx+dy*dy-segment1*segment1-segment2*segment2)/
                            (-2*segment1*segment2));
            return new float[]{targ1/(float)Math.PI,targ2/(float)Math.PI};
        }
    }
}
