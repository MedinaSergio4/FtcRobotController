

package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotHardware {


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor rightFront   = null;
    public DcMotor rightBack  = null;
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor slide = null;
    //public DcMotor inTake = null;

    public static Servo claw = null;
    public static Servo elbow = null;
    public static Servo thrower = null;
    public static Servo hanger = null;

    public static TouchSensor touch = null;




    public static HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public RobotHardware(){}
    // Define a constructor that allows the OpMode to pass a reference to itself.

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap hwmap)    {


        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        rightFront  = hwmap.get(DcMotor.class, "rightFront");
        rightBack = hwmap.get(DcMotor.class, "rightBack");
        leftFront = hwmap.get(DcMotor.class, "leftFront");
        leftBack = hwmap.get(DcMotor.class, "leftBack");
        //inTake = hwmap.get(DcMotor.class, "inTake");
        slide = hwmap.get(DcMotor.class, "slide");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        claw = hwmap.get(Servo.class, "claw");
        elbow = hwmap.get(Servo.class, "elbow");
        thrower = hwmap.get(Servo.class, "thrower");
        hanger = hwmap.get(Servo.class, "hanger");


        touch = hwmap.get(TouchSensor.class, "touch");


        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //inTake.setDirection(DcMotor.Direction.REVERSE);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
/*        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);
*/
    }
}



