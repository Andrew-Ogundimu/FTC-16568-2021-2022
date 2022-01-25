package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.ServoController;

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
                            (-2*segment1*Math.sqrt(dx*dx+dy*dy)))+Math.atan(dy/dx));
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
    private Servo arm1_1 = null;
    private Servo arm1_2 = null;
    private Servo arm2 = null;
    private CRServo wheel = null;
    private Servo grab = null;
    private ServoController sc;
    private Arm total = new Arm(250f,250f);
    private float[] targ_pos = new float[]{105f,60f};
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
        initVuforia();
        initTfod();

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
        arm1_1 = hardwareMap.get(Servo.class, "bottom_arm1");
        arm1_2 = hardwareMap.get(Servo.class,"bottom_arm2");
        arm2 = hardwareMap.get(Servo.class,"top_arm1");
        grab = hardwareMap.get(Servo.class,"grab");
        wheel = hardwareMap.get(CRServo.class,"wheel");
        sc = arm1_1.getController();
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
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
        arm1_2.setDirection(Servo.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Servo Controller",sc.getServoPosition(arm1_1.getPortNumber()));
        telemetry.addData("Servo Controller",sc.getServoPosition(arm1_2.getPortNumber()));
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
    @Override
    public void loop() {
        //arm1_1.setPower(1.0f);
        //arm1_2.setPower(1.0f);
        if (gamepad1.a) {
            grab.setPosition(1.0f);
        } else {
            grab.setPosition(0.0f);
        }

        double m0_power = -gamepad1.left_stick_x; // left or right
        double m2_power = -gamepad1.left_stick_x; // left or right

        double m3_power = gamepad1.left_stick_y; // up or down
        double m1_power = gamepad1.left_stick_y; // up or down

        targ_pos[0]+=(gamepad1.dpad_left ? 1:0)-(gamepad1.dpad_right ? 1:0);
        targ_pos[1]+=(gamepad1.dpad_up ? 1:0)-(gamepad1.dpad_down ? 1:0);
        telemetry.addData("Target (mm)",Float.toString(targ_pos[0])+","+Float.toString(targ_pos[1]));
        if (gamepad1.right_stick_x!=0) {
            m0_power = gamepad1.right_stick_x;
            m1_power = gamepad1.right_stick_x;
            m2_power = -gamepad1.right_stick_x;
            m3_power = -gamepad1.right_stick_x;
        }

        m0.setPower(m0_power);
        m2.setPower(m2_power);

        m3.setPower(m3_power);
        m1.setPower(m1_power);

        telemetry.addData("Test","");

        float[] angles = total.CalcServos(targ_pos[0],targ_pos[1]);
        arm1_1.setPosition((double)angles[0]);
        arm1_2.setPosition((double)angles[0]);
        arm2.setPosition((double)angles[1]);
        telemetry.addData("Angles",Float.toString(angles[0]*180.0f)+" "+Float.toString(angles[1]*180.0f));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gamepad", Float.toString(gamepad1.left_stick_x)+" "+Float.toString(gamepad1.left_stick_y));
        // telemetry.addData("Motors", "left (%.2f), right (%.2f)", horizWheel,vertWheel);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

}
