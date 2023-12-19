package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="park", group="Test")
public class park extends automethods {
    RobotHardware robot = new RobotHardware();// Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();



    //  OpenCvWebcam webcam;
   // testPipeline.BarcodeDeterminationPipeline pipeline;
   // testPipeline.BarcodeDeterminationPipeline.barcodePosition barcodePos;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.autoinit(hardwareMap);

        waitForStart();
      ///////////////////////////////////ROBOT  START////////////////////////////////////////////////////
        robot.leftFront.setPower(.3);
        robot.leftBack.setPower(.3);
        robot.rightBack.setPower(.3);
        robot.rightFront.setPower(.3);
        robot.wrist.setPosition(.3);
        robot.elbow.setPosition(.5);


        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3 )) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();

        }


    }
}
