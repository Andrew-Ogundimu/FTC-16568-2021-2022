package org.firstinspires.ftc.teamcode.teleop;


/**
 * This class is for controlling the robot with the gamepad controller.
 **/

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name="Tele-Op V1", group="Iterative Opmode")
public class Teleop extends OpMode
{
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
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor m0 = null;
    private DcMotor m1 = null;
    private DcMotor m2 = null;
    private DcMotor m3 = null;
    private DcMotor arm1 = null;
    private Servo arm2 = null;
    private DcMotor wheel = null;
    private Servo grab = null;
    final int tickRotation = 1680;
    private Arm total = new Arm(250f,250f);
    private float arm_speed = 3;
    final float[] start_pos = new float[]{105f,60f};
    final double initAngle = (double)(total.CalcServos(start_pos[0],start_pos[1])[0]*180);
    private float[] targ_pos = start_pos.clone();
    private float[] last_targ = targ_pos.clone();
    private double hFOV = 60; //Horizontal FOV in degrees
    private double vFOV; //vertical FOV in degrees
    private double focal_length; //focal length in pixels
    private double focal_mm = 4; //focal length in mm
    private Map<String,Double> object_sizes = new HashMap<>(); //size of Ball, Cube, Duck, Marker in that order in mm

    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AafM8Wb/////AAABmZTMSdMtfkaviYDOO0/st50G9epDv7Zab2Z4LYeKWEIr6VkdnGknUhoExT91we7eHphM+Z+t6MZHvnB4Gfl7Zt6HqkN2LFPsR0hE8PxYcQaqvxUgZ2iypyRRm833itT7K3ewaiuIkMVpTsTh1K1YrgPxHY60jUAvPdlIJnbtQZGlGTAD1oSOdtd4JAqSujxoApI5cszs/xLWPWkOjQkzVN+HBdAVPCgLh67MAc96xzUH7+NRhq6omjxGN9wRbAl3LeF9sCIWB7pvXtuTKSw5zHUIZA7wLW2J9iFEzt1KVo0gUGxYC6GQi2JOF0qQKTWFVWPC2LKF6qRncxjLW2qbDhfkMqlUIfAsmxMqmp8yQJGp";

    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //initVuforia();
        //initTfod();

        object_sizes.put("Ball",69.9);
        object_sizes.put("Cube",50.8);
        object_sizes.put("Duck",51.86); //average of the width,length, and height


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        m0 = hardwareMap.get(DcMotor.class, "m0"); // fl
        m1 = hardwareMap.get(DcMotor.class, "m1"); // fr
        m2 = hardwareMap.get(DcMotor.class, "m2"); // bl
        m3 = hardwareMap.get(DcMotor.class, "m3"); // br
        arm1 = hardwareMap.get(DcMotor.class, "bottom_arm1");
        arm2 = hardwareMap.get(Servo.class,"top_arm1");
        grab = hardwareMap.get(Servo.class,"grab");
        wheel = hardwareMap.get(DcMotor.class,"wheel");
        //arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setDirection(DcMotor.Direction.FORWARD);
        m0.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.REVERSE);

        m1.setDirection(DcMotor.Direction.FORWARD);
        m2.setDirection(DcMotor.Direction.REVERSE);

        int image_width = 1280;
        int image_height = 720;
        focal_length = image_width/(2*Math.tan(hFOV/2));
        vFOV = 2*Math.atan(0.5*image_height/focal_length);
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //sc.setServoPosition(arm1_1.getPortNumber(),0.25);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    public int positive (double turn) {
        if (turn!=0) {
            return (int)(turn/Math.abs(turn));
        } else {
            return 1;
        }
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public float[] rotate(float[] point,float degrees) {
        double d = Math.toDegrees((double)degrees);
        return new float[]{(float)(Math.cos(d)*point[0]-Math.sin(d)*point[1]),(float)(Math.cos(d)*point[1]+Math.sin(d)*point[0])};
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
    @Override
    public void loop() {
        arm1.setPower(1.0f);
        if(gamepad1.a) {
            wheel.setPower(1.0);
        } else if (gamepad1.b) {
            wheel.setPower(-1.0);
        } else {
            wheel.setPower(0);
        }
        if (gamepad1.y) {
            targ_pos = start_pos.clone();
        }
        if (gamepad2.x) {
            targ_pos = new float[]{357,215};
        }

        telemetry.addData("Start:",Float.toString(start_pos[0])+" "+Float.toString(start_pos[1]));
        telemetry.addData("Motor Pos:",Integer.toString(arm1.getCurrentPosition()));
        if (gamepad1.right_trigger>0) {
            grab.setPosition(0.0f);
        } else {
            grab.setPosition(1.0f);
        }

        float[] move_vec = new float[]{gamepad1.left_stick_x,gamepad1.left_stick_y};
        float[] r = rotate(move_vec,-45); //rotate the movement vector by 45 degrees
        double m0_power = -r[0]; // left or right
        double m2_power = -r[0]; // left or right

        double m3_power = r[1]; // up or down
        double m1_power = r[1]; // up or down

        targ_pos[0]+=(gamepad1.left_trigger > 0 ?1:0)*arm_speed-(gamepad1.left_bumper ? 1:0)*arm_speed;
        targ_pos[1]+=(gamepad1.dpad_up ? 1:0)*arm_speed-(gamepad1.dpad_down ? 1:0)*arm_speed;
        if ((Math.sqrt(targ_pos[0]*targ_pos[0]+targ_pos[1]*targ_pos[1])>total.segment1+total.segment2) || (Arrays.asList(targ_pos).contains(null))) {
            targ_pos = last_targ.clone();
        }
        telemetry.addData("Target (mm)",Float.toString(targ_pos[0])+","+Float.toString(targ_pos[1]));
        if (gamepad1.right_stick_x!=0) {
            m0_power = -gamepad1.right_stick_x;
            m1_power = -gamepad1.right_stick_x;
            m2_power = gamepad1.right_stick_x;
            m3_power = gamepad1.right_stick_x;
        }

        m0.setPower(m0_power);
        m2.setPower(m2_power);

        m3.setPower(m3_power);
        m1.setPower(m1_power);

        telemetry.addData("Test","");

        float[] angles = total.CalcServos(targ_pos[0],targ_pos[1]);
        arm2.setPosition(clip(1.0-(double)angles[1]));
        arm1.setTargetPosition((int)(-((angles[0])*(tickRotation/2)-initAngle/180*tickRotation/2))); //some function that implements angles[0]
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Angles",Float.toString(angles[0]*180.0f)+" "+Float.toString(angles[1]*180.0f));
        telemetry.addData("motor",arm1.getCurrentPosition());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gamepad", Float.toString(gamepad1.left_stick_x)+" "+Float.toString(gamepad1.left_stick_y));
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", horizWheel,vertWheel);
        last_targ = targ_pos.clone();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
