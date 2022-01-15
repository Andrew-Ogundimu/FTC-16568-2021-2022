package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BotMovement {
    final float WHEEL_RADIUS = 5.08f; //wheel radius in cm
    final float BOT_RADIUS = 26.3f; //bot radius in cm

    public BotMovement(DcMotor[] motors) {
        motors[0].getCurrentPosition();
    }
    public void rotate(float angle) {

    }
    public void move(float dx, float dy, float dz) {

    }
}
