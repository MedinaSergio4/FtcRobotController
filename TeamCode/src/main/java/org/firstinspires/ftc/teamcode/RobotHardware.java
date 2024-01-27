

package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    public static IMU imu;// Additional Gyro device



    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor slide;
    public DcMotor hanger;
    //public DcMotor inTake = null;

    public static Servo claw1 = null;
    public static Servo elbow = null;
    public  static Servo claw2 = null;
    public static Servo thrower = null;
    //public static Servo hanger = null;
    public static Servo hangClaw = null;

    //public static TouchSensor touch = null;




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

        imu = hwmap.get(IMU.class, "imu");

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        rightFront  = hwmap.get(DcMotor.class, "rightFront");
        rightBack = hwmap.get(DcMotor.class, "rightBack");
        leftFront = hwmap.get(DcMotor.class, "leftFront");
        leftBack = hwmap.get(DcMotor.class, "leftBack");
        //inTake = hwmap.get(DcMotor.class, "inTake");
        slide = hwmap.get(DcMotor.class, "slide");
        hanger = hwmap.get(DcMotor.class, "hanger");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        claw1 = hwmap.get(Servo.class, "claw1");
        elbow = hwmap.get(Servo.class, "elbow");
        claw2 = hwmap.get(Servo.class, "claw2");
        thrower = hwmap.get(Servo.class, "thrower");
        hangClaw = hwmap.get(Servo.class, "hangClaw");
        //hanger = hwmap.get(Servo.class, "hanger");


        //touch = hwmap.get(TouchSensor.class, "touch");



        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);
        hanger.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hanger.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hanger.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        thrower.setPosition(.125);
        claw1.setPosition(.6);
        claw2.setPosition(.65);



    }
    public static void autoinit (HardwareMap ahwMap){



    }
}



