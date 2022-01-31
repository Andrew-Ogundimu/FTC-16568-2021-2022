package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BotMovement {
    final float WHEEL_RADIUS = 50.8f; //wheel radius in mm
    final float BOT_RADIUS = 263f; //bot radius in mm

    public BotMovement(DcMotor[] motors) {
        motors[0].getCurrentPosition();
    }
    public void rotate(float angle) {

    }
    public void move(float dx, float dy, float dz) {

    }
}
