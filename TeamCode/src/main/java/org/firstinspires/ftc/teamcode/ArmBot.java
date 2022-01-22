/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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


@TeleOp(name="ArmBot Tele-Op", group="Iterative Opmode")
public class ArmBot extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private Servo arm1 = null;
    private Servo arm2 = null;
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
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
<<<<<<< Updated upstream
        fl = hardwareMap.get(DcMotor.class, "front_left");
        fr = hardwareMap.get(DcMotor.class, "front_right");
        bl = hardwareMap.get(DcMotor.class, "back_left");
        br = hardwareMap.get(DcMotor.class, "back_right");
=======
        br = hardwareMap.get(DcMotor.class, "1"); //back right
        bl = hardwareMap.get(DcMotor.class, "2"); //back left
        fl = hardwareMap.get(DcMotor.class, "3"); //front left
        fr = hardwareMap.get(DcMotor.class, "4"); //front right
>>>>>>> Stashed changes
        arm1 = hardwareMap.get(Servo.class,"arm1");
        arm2 = hardwareMap.get(Servo.class,"arm2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
<<<<<<< Updated upstream
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
=======
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);

        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
>>>>>>> Stashed changes

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
        List<Recognition> objects = tfod.getUpdatedRecognitions();

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        int num = 0;
        if (objects != null) {
            for (Recognition rec : objects) {
                num++;
                if (rec.getLabel() != "Marker") {
                    double distance = focal_mm * object_sizes.get(rec.getLabel()) / rec.getWidth();
                    telemetry.addData("Object " + String.valueOf(num) + " - (" + rec.getLabel() + ")", "Distance: " + String.valueOf(distance));
                }
            }
        }
        double power1 = gamepad1.left_stick_x;
        double power2 = gamepad1.left_stick_y;
<<<<<<< Updated upstream
        double power3 = -gamepad1.left_stick_x;
        double power4 = -gamepad1.left_stick_y;

        fl.setPower(power1);
        br.setPower(power2);
        bl.setPower(power3);
=======
        double power3 = gamepad1.left_stick_x;
        double power4 = gamepad1.left_stick_y;
        if (gamepad1.right_stick_x!=0) {
            power1 = gamepad1.right_stick_x;
            power2 = gamepad1.right_stick_x;
            power3 = -gamepad1.right_stick_x;
            power4 = -gamepad1.right_stick_x;
        }

        br.setPower(power1);
        bl.setPower(power2);
        fl.setPower(power3);
>>>>>>> Stashed changes
        fr.setPower(power4);

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
