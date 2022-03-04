package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Settings;
import org.firstinspires.ftc.teamcode.teleop.Teleop_With_Camera;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.lang.Math;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * State behavior: moves the arm to a specified location.
 */

public class NewArmState extends State {

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
    private CRServo grab = null;
    private double servo_range = 280;
    OpenCvWebcam webcam;
    // NewArmState.SkystoneDeterminationPipeline pipeline;
    final int tickRotation = 1680;
    private NewArmState.Arm total = new NewArmState.Arm(250f,250f);
    private float arm_speed = 3.5f;
    final float[] start_pos = new float[]{105f,60f};
    final double initAngle = (double)(total.CalcServos(start_pos[0],start_pos[1])[0]*180);
    private float[] targ_pos = start_pos.clone();
    private float[] last_targ = targ_pos.clone();
    final float[] grab_pos = new float[]{389f,-54f};
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

    private int pos_x = 0;
    private int pos_y = 0;
    private int height = 0;
    private Telemetry telemetry;
    private float[] angles;

    //new method for beta PID-drive
    public NewArmState(int pos_x, int pos_y, HardwareMap hardwareMap, Telemetry telemetry) {
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

        //initVuforia();
        //initTfod();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // pipeline = new Teleop_With_Camera.SkystoneDeterminationPipeline();
        // webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

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
        grab = hardwareMap.get(CRServo.class,"grab");
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

    @Override
    public void update() {
        arm1.setPower(0.8f);

        /**
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
         */

        telemetry.addData("Start:",Float.toString(start_pos[0])+" "+Float.toString(start_pos[1]));
        telemetry.addData("Motor Pos:",Integer.toString(arm1.getCurrentPosition()));

        /**
        if (gamepad1.right_trigger>0) {
            grab.setPower(1.0);
        } else if (gamepad1.right_bumper){
            grab.setPower(-1.0);
        } else {
            grab.setPower(0.0);
        }
         */

        /**
        if (gamepad1.x) {
            targ_pos = grab_pos.clone();
        }

        float[] move_vec = new float[]{gamepad1.left_stick_x,gamepad1.left_stick_y};
        float[] r = rotate(move_vec,-45); //rotate the movement vector by 45 degrees
        double m0_power = -r[0]; // left or right
        double m2_power = -r[0]; // left or right

        double m3_power = r[1]; // up or down
        double m1_power = r[1]; // up or down
         */

        targ_pos[0] = pos_x;
        targ_pos[1] = pos_y;

        if ((Math.sqrt(targ_pos[0]*targ_pos[0]+targ_pos[1]*targ_pos[1])>total.segment1+total.segment2) || (Arrays.asList(targ_pos).contains(null))) {
            targ_pos = last_targ.clone();
        }
        telemetry.addData("Target (mm)",Float.toString(targ_pos[0])+","+Float.toString(targ_pos[1]));

        /**
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
         */

        //telemetry.addData("Test","");
        angles = total.CalcServos(targ_pos[0],targ_pos[1]);
        arm2.setPosition(clip(angles[1]*180/servo_range+(servo_range-180)/servo_range));
        arm1.setTargetPosition((int)(((angles[0])*(tickRotation/2)-initAngle/180*tickRotation/2))); //some function that implements angles[0]
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Angles",Float.toString(angles[0]*180.0f)+" "+Float.toString(angles[1]*180.0f));
        telemetry.addData("motor",arm1.getCurrentPosition());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Gamepad", Float.toString(gamepad1.left_stick_x)+" "+Float.toString(gamepad1.left_stick_y));
        //telemetry.addData("Marker:",pipeline.getAnalysis());
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", horizWheel,vertWheel);
        last_targ = targ_pos.clone();
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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public float[] rotate(float[] point,float degrees) {
        double d = Math.toRadians((double)degrees);
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
}
