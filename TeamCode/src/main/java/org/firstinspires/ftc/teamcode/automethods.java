package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.concurrent.TimeUnit;


@Autonomous(name="A", group="Park")

/* This autonomous program is designed to go forward, pick up a stone, and deliver it to the blue tray, before returning and repeating it once more.*/
public class automethods extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    ElapsedTime timer = new ElapsedTime();
    String slideDataSTR;
    int slideDown;
    int slideTarget;
    double pusherPushing = .09;
    double pusherClose = .2;

    double doorClose= 0.7;
    double doorOpen =1;

    double twisterDeliver=0.4;
    double twisterNeutral=0.815;



    /* Declare OpMode members. */
///////////////////////////////////wheel calibration//////////////////////////
    RobotHardware robot = new RobotHardware();   // Use a Pushbot's hardware


    static final double COUNTS_PER_MOTOR_REV = 537.7;    //need to adjust for big wheels
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.2;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight ss we can make it with an integer gyro
    static final double P_TURN_COEFF = .1;     // Larger is more responsive, but also less stable
    // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
    }


    public void encoderDrive(double speed, double inches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        /////////////////////////////Moving straight ///////////////////////////////////////////////
        if (opModeIsActive()) {
           // robot.cannon.setPower(.86);//////necessary to shoot accurately///////////

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);

            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightBack.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d    : %7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d    : %7d:%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }

    }

    ////////////////////////////arm up init//////////////////////////////


public void startturn(double speed, double timeoutS)
{
    if (opModeIsActive()) {
        // reset the timeout time and start motion.
        runtime.reset();
      //  robot.turntableLeft.setPower(speed);
        //robot.turntableRight.setPower(speed);
        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {
            // Display it for the driver.
            telemetry.addData("Running", "True");
            telemetry.update();
        }
        // Stop all motion;
     //   robot.turntableLeft.setPower(0);
       // robot.turntableRight.setPower(0);
    }}

    //////////////////////////turning////////////////////
    public void imuTurn(double speed, double angle) {

        // keep looping while we are still actifve, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            leftSpeed = speed * steer;
            rightSpeed = -leftSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    //////////////////////////////////////Barcode////////////////

    public double getError(double targetAngle) {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - orientation.getYaw(AngleUnit.DEGREES);
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    ///////////////////////////////////////////move towards parking space///////////////////////
    public void strafeRight(double speed, double inches,
                            double timeoutS) {
       // robot.cannon.setPower(.86);

        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller//////////this is where you change direction
            newLeftFrontTarget = robot.rightFront.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (-inches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);


            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("StrafePath1", "Running to %7d :%7d    : %7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("StrafePath2", "Running at %7d :%7d    : %7d:%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }



    public void imuHold(double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            // onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        // robot.wobble.setPower(0);

        robot.slide.setPower(0);

    }
/*
    public void setLevel(double level){
    if (level == 1){
        slideTarget = slideDown-1000;
        slideDataSTR = "BOTTOM";
        robot.slide.setTargetPosition(slideTarget);
        //move the slide
        robot.door.setPosition(doorClose);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(.9);

        while (robot.slide.isBusy()){
            telemetry.addData("SLIDE", "running to %7d : %7d",
                    slideTarget,
                    robot.slide.getCurrentPosition());
            //telemetry.addData(slideDataSTR);
            telemetry.update();
        }
        robot.slide.setPower(0);
        robot.pusher.setPosition(pusherClose);
        robot.door.setPosition(doorClose);
        robot.twister.setPosition(twisterDeliver);

        timer.reset();
        while(timer.time(TimeUnit.MILLISECONDS) < 300){
            robot.pusher.setPosition(pusherPushing);
            robot.door.setPosition(doorOpen);
        }
    }

    else if (level == 2){
        slideTarget = slideDown-1300;
        slideDataSTR = "MIDDLE";
        robot.slide.setTargetPosition(slideTarget);
        //move the slide
        robot.door.setPosition(doorClose);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(.9);

        while (robot.slide.isBusy()){
            telemetry.addData("SLIDE", "running to %7d : %7d",
                    slideTarget,
                    robot.slide.getCurrentPosition());
            //telemetry.addData(slideDataSTR);
            telemetry.update();
        }
        robot.slide.setPower(0);
        robot.pusher.setPosition(pusherClose);
        robot.door.setPosition(doorClose);
        robot.twister.setPosition(twisterDeliver);

        timer.reset();
        while(timer.time(TimeUnit.MILLISECONDS) < 300){
            robot.pusher.setPosition(pusherPushing);
            robot.door.setPosition(doorOpen);
        }
    }
    else if (level == 3){
        slideTarget = slideDown-1509;
        slideDataSTR = "TOP";
        robot.slide.setTargetPosition(slideTarget);
        //move the slide
        robot.door.setPosition(doorClose);
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(.9);

        while (robot.slide.isBusy()){
            telemetry.addData("SLIDE", "running to %7d : %7d",
                    slideTarget,
                    robot.slide.getCurrentPosition());
            //telemetry.addData(slideDataSTR);
            telemetry.update();
        }
        robot.slide.setPower(0);
        robot.pusher.setPosition(pusherClose);
        robot.door.setPosition(doorClose);
        robot.twister.setPosition(twisterDeliver);

        timer.reset();
        while(timer.time(TimeUnit.MILLISECONDS) < 300){
            robot.pusher.setPosition(pusherPushing);
            robot.door.setPosition(doorOpen);
        }
    }

    robot.slide.setTargetPosition(slideTarget);
    //move the slide

        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.slide.isBusy()){
            telemetry.addData("SLIDE", "running to %7d : %7d",
                    slideTarget,
                    robot.slide.getCurrentPosition());
            //telemetry.addData(slideDataSTR);
            telemetry.update();
        }


    }
    public void setLevelDown(int slideTarget){
        timer.reset();
        while(timer.time(TimeUnit.MILLISECONDS) < 1000){
            robot.pusher.setPosition(pusherClose);
            robot.door.setPosition(doorClose);
            robot.twister.setPosition(twisterNeutral);
        }

        robot.slide.setTargetPosition(slideTarget);
        //move the slide


        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slide.setPower(-.9);

        while (robot.slide.isBusy()){
            telemetry.addData("SLIDE", "running to %7d : %7d",
                    slideTarget,
                    robot.slide.getCurrentPosition());
            //telemetry.addData(slideDataSTR);
            telemetry.update();
        }
        robot.slide.setPower(0);
        robot.door.setPosition(doorOpen);

    }*/

}