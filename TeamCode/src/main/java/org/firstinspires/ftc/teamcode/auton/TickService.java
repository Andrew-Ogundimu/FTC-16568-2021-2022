package org.firstinspires.ftc.teamcode.auton;

public final class TickService {

    private static final double wheelCircumference = (2.0 * 2) * Math.PI;
    private static final int ticksPerTurn = 1680;

    public static int inchesToTicks(double inches) {
        double circumferenceTraveled = inches / wheelCircumference;
        int ticksTraveled = (int) (ticksPerTurn * circumferenceTraveled);

        return ticksTraveled;
    }
}
