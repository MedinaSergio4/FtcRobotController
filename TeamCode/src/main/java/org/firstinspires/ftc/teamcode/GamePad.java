package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;


import java.util.concurrent.TimeUnit;


@TeleOp(name = "Gamepad0234", group = "Linear Opmode")
public class GamePad extends OpMode {
    RobotHardware robot = new RobotHardware();
    ElapsedTime timer = new ElapsedTime();

    double robotAngle;
    double rightX;
    double h;
    double pmodify = .35;

    double LeftFrontPower;
    double LeftBackPower;
    double RightFrontPower;
    double RightBackPower;

    int slideDown;

    boolean squarePressed;
    boolean trianglePressed;
    boolean circlePressed;
    boolean crossPressed;

    int clawCount;
    int elbowCount;
    int throwerCount;
    int hangerCount;

    @Override
    public void init() {
        robot.init(hardwareMap);
        slideDown = robot.slide.getCurrentPosition();
        //robot.wrist.setPosition(.375);
        //robot.elbow.setPosition(.6775);


        }




    @Override
    public void loop() {
        getRuntime();

//////////////////////////drive system
        //telemetry.addData("SLIDE", "Current Position: %7d", robot.slide.getCurrentPosition());
        //telemetry.update();

        h = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        rightX = gamepad1.right_stick_x;

        LeftFrontPower = h * Math.sin(robotAngle) - rightX;
        RightFrontPower = h * Math.cos(robotAngle) + rightX;
        LeftBackPower = h * Math.cos(robotAngle) - rightX;
        RightBackPower = h * Math.sin(robotAngle) + rightX;

        //SLOW MODE
        if(gamepad1.right_trigger > .5) {
            pmodify = .25;// Slower Speed
        }

        else{
            pmodify = 1;//Normal Speed
        }

        robot.leftFront.setPower(LeftFrontPower * pmodify);
        robot.rightFront.setPower(RightFrontPower * pmodify);
        robot.leftBack.setPower(LeftBackPower * pmodify);
        robot.rightBack.setPower(RightBackPower * pmodify);


        telemetry.addData("throwerValue", robot.thrower.getPosition());


/////////////////////////////////////slide
/* Servo Example
        if(gamepad1.circle ){
            robot.claw.setPosition(.275);
            try {
                wait(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setLevelUp(slideDown - 2100);
        }
        if (gamepad1.triangle ){
            robot.claw.setPosition(.05);
        }




///// Set levels
        if (gamepad1.dpad_down ) {//floor
            setLevelDown(slideDown + 23);

        }

        if (gamepad1.dpad_left) {//low
            setLevelDown(slideDown - 2100);
        }


        if (gamepad1.dpad_up) {/////////medium
            setLevelUp(slideDown - 3666);
        }

        if(gamepad1.dpad_right){
            setLevelDown(slideDown - 150);
        }
*/
/////////////manual level set
        if(gamepad1.left_bumper && robot.slide.getCurrentPosition()<=2220) {
            robot.slide.setPower(.6);///up
        }
        else if(gamepad1.left_trigger >  .5 ){///down
            robot.slide.setPower(-.6);//down
        }
        else{
            robot.slide.setPower(0);//stay still
        }



        if (gamepad1.square){
            robot.claw1.setPosition(.6);
            robot.claw2.setPosition(.65);
        }///close
        if (gamepad1.cross){
            robot.claw1.setPosition(0.4);
            robot.claw2.setPosition(.75);
        }/////open

        if (gamepad1.dpad_down){
            //robot.wrist.setPosition(.375);
            robot.elbow.setPosition(.435);

        }///down
        if (gamepad1.dpad_up){
            //robot.wrist.setPosition(.3);
            robot.elbow.setPosition(.75);

        }///up

 /*       if(gamepad1.triangle){
            robot.thrower.setPosition(.125);
        }///up*/
        if(gamepad1.circle){
            robot.thrower.setPosition(-.125);
        }/////down, thrown
        if(gamepad1.dpad_left){
        robot.hanger.setPower(.6);
        }/////up
        else if(gamepad1.dpad_right) {
            robot.hanger.setPower(-.6);
        }////down
        else{robot.hanger.setPower(0);}

        if(gamepad1.right_bumper){
            robot.hangClaw.setPosition(0.05);
        }//////uppp
       /* if(gamepad1.share){
            robot.hangClaw.setPosition(.05);
        }////up but down a little*/
        if(gamepad1.touchpad){
            robot.hangClaw.setPosition(.33);
        }//////down

        //0 has hanger right up against the top, and .05 is just a little after that

        if (gamepad1.triangle && getRuntime()>500){
            robot.thrower.setPosition(.125);
        }

        telemetry.addData("claw1Pos","%.4f",robot.claw1.getPosition());
        telemetry.addData("claw2Pos","%.4f",robot.claw2.getPosition());
        telemetry.addData("throwerPos","%.4f",robot.thrower.getPosition());
        telemetry.addData("elbowPos","%.4f",robot.elbow.getPosition());

        telemetry.addData("Runtime", getRuntime());

        telemetry.addData("slidePos",robot.slide.getCurrentPosition());

        telemetry.update();

        /*
        if (gamepad1.square) {
            if (!squarePressed) {
                clawCount += 1;
                squarePressed = true;
            } else {
                squarePressed = false;
            }

            if (clawCount % 2 == 0) {
                robot.claw.setPosition(.86);
            } else {
                robot.claw.setPosition(0);
            }
        }*/



       /* if (gamepad1.triangle) {
            if (!trianglePressed) {
                elbowCount += 1;
                trianglePressed = true;
            } else {
                trianglePressed = false;
            }

            if (elbowCount % 2 == 0) {
                robot.elbow.setPosition(.35);
            } else {
                robot.elbow.setPosition(.1);
            }
        }*/



        /*if (gamepad1.circle) {
            if (!circlePressed) {
                hangerCount += 1;
                circlePressed = true;
            } else {
                circlePressed = false;
            }

            if (hangerCount % 2 == 0) {
                robot.hanger.setPosition(.86);
            } else {
                robot.hanger.setPosition(0);
            }
        }

        if (gamepad1.cross) {
            if (!crossPressed) {
                throwerCount += 1;
                crossPressed = true;
            } else {
                crossPressed = false;
            }

            if (throwerCount % 2 == 0) {
                robot.thrower.setPosition(.86);
            } else {
                robot.thrower.setPosition(0);
            }
        }
        */
        //////////////////////////////////////Levels

    }

    public void setLevelUp(int slideTarget){


        robot.slide.setTargetPosition(slideTarget);
        //move the slide
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
        robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setLevelDown(int slideTarget){

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
        robot.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }}

