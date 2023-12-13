package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    @Override
    public void init() {
        robot.init(hardwareMap);
        slideDown = robot.slide.getCurrentPosition();

    }

    @Override
    public void loop() {
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
        if(gamepad1.left_bumper) {
            robot.slide.setPower(-.6);///up
        }
        else if(gamepad1.left_trigger >  .5 ){///down
            robot.slide.setPower(.6);//down
        }
        else{
            robot.slide.setPower(0);//stay still
        }




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

