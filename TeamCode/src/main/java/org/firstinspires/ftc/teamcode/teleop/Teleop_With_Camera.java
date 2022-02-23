package org.firstinspires.ftc.teamcode.teleop;


/**
 * This class is for controlling the robot with the gamepad controller.
 **/

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Tele-Op Camera V1", group="Iterative Opmode")
public class Teleop_With_Camera extends OpMode
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

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181,98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253,98);
        static final int REGION_WIDTH = 20;
        static final int REGION_HEIGHT = 20;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SkystoneDeterminationPipeline.SkystonePosition position = SkystoneDeterminationPipeline.SkystonePosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.min(avg1, avg2);
            int max = Math.min(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = SkystoneDeterminationPipeline.SkystonePosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg2) // Was it from region 2?
            {
                position = SkystoneDeterminationPipeline.SkystonePosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = SkystoneDeterminationPipeline.SkystonePosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public SkystoneDeterminationPipeline.SkystonePosition getAnalysis()
        {
            return position;
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
    private double servo_range = 180;
    OpenCvWebcam webcam;
    SkystoneDeterminationPipeline pipeline;
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
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

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
        if (gamepad1.x) {
            if (pipeline.getAnalysis() == SkystoneDeterminationPipeline.SkystonePosition.LEFT) {
                targ_pos = new float[]{262, 370};
            } else if (pipeline.getAnalysis() == SkystoneDeterminationPipeline.SkystonePosition.CENTER) {
                targ_pos = new float[]{262, 265};
            } else if (pipeline.getAnalysis() == SkystoneDeterminationPipeline.SkystonePosition.RIGHT) {
                targ_pos = new float[]{262, 153};
            }
        }

        telemetry.addData("Start:",Float.toString(start_pos[0])+" "+Float.toString(start_pos[1]));
        telemetry.addData("Motor Pos:",Integer.toString(arm1.getCurrentPosition()));
        if (gamepad1.right_trigger>0) {
            grab.setPower(1.0);
        } else {
            grab.setPower(0.0);
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
        arm2.setPosition(clip((double)angles[1])*180/servo_range);
        arm1.setTargetPosition((int)(((angles[0])*(tickRotation/2)-initAngle/180*tickRotation/2))); //some function that implements angles[0]
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Angles",Float.toString(angles[0]*180.0f)+" "+Float.toString(angles[1]*180.0f));
        telemetry.addData("motor",arm1.getCurrentPosition());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gamepad", Float.toString(gamepad1.left_stick_x)+" "+Float.toString(gamepad1.left_stick_y));
        telemetry.addData("Marker:",pipeline.getAnalysis());
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