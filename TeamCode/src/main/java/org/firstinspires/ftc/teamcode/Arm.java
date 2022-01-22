<<<<<<< Updated upstream
package org.firstinspires.ftc.teamcode;

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
}
=======
package org.firstinspires.ftc.teamcode;

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
        return new float[]{targ1,targ2};
    }
}
>>>>>>> Stashed changes
