package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="autonomous", group="Test")
public class autonomous extends automethods {
    RobotHardware robot = new RobotHardware();// Use a Pushbot's hardware


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       /* webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new BarcodePositionDetector.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvWebcam.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        robot.init(hardwareMap);
     //   robot.autoinit(hardwareMap);
       /* int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
*/


        waitForStart();

////////////////////////////////////ROBOT  START////////////////////////////////////////////////////


        //webcam
        strafeRight(.5,60,6);
        imuHold(0.3);
        startturn(1,4);
        strafeRight(.5, -120,8);
        encoderDrive(.5, 40, 5);
        ///deliver freight
        encoderDrive(.5, -40, 5);
        strafeRight(.5, -60, 5);

        //  rpos = returnRingPosition(3);




    }}