package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotHardware;


import java.util.concurrent.TimeUnit;
@TeleOp(name = "ServoTest", group = "Linear Opmode")

public class servoTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware();
        ElapsedTime timer = new ElapsedTime();
        while(!isStopRequested()){
            telemetry.addData("claw1Pos","%.4f",robot.claw1.getPosition());
            telemetry.addData("claw2Pos","%.4f",robot.claw2.getPosition());
            telemetry.addData("throwerPos","%.4f",robot.thrower.getPosition());
            telemetry.addData("elbowPos","%.4f",robot.elbow.getPosition());
            telemetry.update();
        }

    }
}
