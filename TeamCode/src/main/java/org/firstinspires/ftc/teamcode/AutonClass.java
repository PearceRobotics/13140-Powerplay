package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

public abstract class AutonClass extends LinearOpMode {
    // Declare Devices
    DcMotor frontleft = null;
    DcMotor frontright = null;
    DcMotor backleft = null;
    DcMotor backright = null;
    DcMotor rightArm = null;
    DcMotor leftArm = null;
    Servo leftIntake = null;
    Servo rightIntake = null;

    double targetFound;



    // drive motor position variables
    private int lfPos; private int rfPos; private int lrPos; private int rrPos;
    private int leftArmPos; private int rightArmPos;

    // operational constants
    private double fast = .75; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.5; // medium speed
    private double slow = 0.30; // slow speed
    private double clicksPerInch = 44.56; // empirically measured 4x encoding
    private double clicksPerDeg = 9.02; // empirically measured 4x encoding
    private double tol = .1 * clicksPerInch;
    // 8.25 inches is wheels relative to center of robot
    // 8.13 inches is wheels relative to center of robot
    //private double 45 = 90 * 9.45 - 570.6;


    public AutonClass() {}


   public double vision(){

       // April Tags decleration
       OpenCvCamera camera;
       AprilTagDetectionPipeline aprilTagDetectionPipeline;

       double FEET_PER_METER = 3.28084;

       // Lens intrinsics
       // UNITS ARE PIXELS
       // NOTE: this calibration is for the C920 webcam at 800x448.
       // You will need to do your own calibration for other configurations!
       double fx = 578.272;
       double fy = 578.272;
       double cx = 402.145;
       double cy = 221.506;

       // UNITS ARE METERS
       double tagsize = 0.166;

       int numFramesWithoutDetection = 0;

       final float DECIMATION_HIGH = 3;
       final float DECIMATION_LOW = 2;
       final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
       final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;


       AprilTagDemo aprilTagDemo;

       // April Tags initialization
       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       // TODO
       camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
       aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

       camera.setPipeline(aprilTagDetectionPipeline);
       camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
       {
           @Override
           public void onOpened()
           {
               // TODO
               camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
           }

           @Override
           public void onError(int errorCode)
           {

           }
       });
       // Calling getDetectionsUpdate() will only return an object if there was a new frame
       // processed since the last time we called it. Otherwise, it will return null. This
       // enables us to only run logic when there has been a new frame, as opposed to the
       // getLatestDetections() method which will always return an object.
       ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

       //
       if (detections != null) {
           telemetry.addData("FPS", camera.getFps());
           telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
           telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());
           telemetry.addData("Tag Count", detections.size());

           // If we don't see any tags
           if (detections.size() == 0) {
               numFramesWithoutDetection++;

               // If we haven't seen a tag for a few frames, lower the decimation
               // so we can hopefully pick one up if we're e.g. far back
               if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                   aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
               }
           }
           // We do see tags!
           else {
               numFramesWithoutDetection = 0;

               // If the target is within 1 meter, turn on high decimation to
               // increase the frame rate
               if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                   aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
               }

               for (AprilTagDetection detection : detections) {
                   telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                   telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
                   telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
                   telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
                   telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
                   telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
                   telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
               }

           }

           telemetry.update();
       }

       sleep(20);

       while(detections.size() == 0) {
           targetFound = 0;

           if (detections.size() >= 1) {
               targetFound = detections.get(0).id;
               break;
           }
       }
    return targetFound;
   }

    public void moveForward(int howMuch, double speed) {
        backleft = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        backright = hardwareMap.get(DcMotor.class, "BackRightWheel");
        frontleft = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        frontright = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        // The right motors need reversing
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        // howMuch is in inches. A negative howMuch moves backward.
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setTargetPosition(0);
        frontleft.setTargetPosition(0);
        backleft.setTargetPosition(0);
        backright.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // fetch motor positions
        lfPos = frontleft.getCurrentPosition();
        rfPos = frontright.getCurrentPosition();
        lrPos = backleft.getCurrentPosition();
        rrPos = backright.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lrPos += howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);
        // move robot to new position
        frontleft.setTargetPosition(lfPos);
        frontright.setTargetPosition(rfPos);
        backleft.setTargetPosition(lrPos);
        backright.setTargetPosition(rrPos);

        // wait for move to complete
        while (frontleft.isBusy() || frontright.isBusy() ||
                backleft.isBusy() || backright.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Move Foward");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d",
                    frontleft.getCurrentPosition(),
                    frontright.getCurrentPosition(),
                    backleft.getCurrentPosition(),
                    backright.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

    public void moveRight(int howMuch, double speed) {
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = frontleft.getCurrentPosition();
        rfPos = frontright.getCurrentPosition();
        lrPos = backleft.getCurrentPosition();
        rrPos = backright.getCurrentPosition();

        // calculate new targets
        lfPos += howMuch * clicksPerInch;
        rfPos -= howMuch * clicksPerInch;
        lrPos -= howMuch * clicksPerInch;
        rrPos += howMuch * clicksPerInch;

        // move robot to new position
        frontleft.setTargetPosition(lfPos);
        frontright.setTargetPosition(rfPos);
        backleft.setTargetPosition(lrPos);
        backright.setTargetPosition(rrPos);
        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);

        // wait for move to complete
        while (frontleft.isBusy() && frontright.isBusy() &&
                backleft.isBusy() && backright.isBusy()) {

            // Display it for the driver.
            telemetry.addLine("Strafe Right");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d",frontleft.getCurrentPosition(),
                    frontright.getCurrentPosition(),backleft.getCurrentPosition(),
                    backright.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

    }
    private void intakePosition(int howMuch, double speed) {
        // Initialize the servos for intake
        leftIntake = hardwareMap.get(Servo.class, "Intake");
        rightIntake = hardwareMap.get(Servo.class, "Intake2");

        leftIntake.setDirection(Servo.Direction.FORWARD);
        rightIntake.setDirection(Servo.Direction.REVERSE);


        leftIntake.getPosition();
        rightIntake.getPosition();
        leftIntake.setPosition((int) (howMuch * clicksPerInch));
        rightIntake.setPosition((int) (howMuch * clicksPerInch));

        while (leftIntake.getPosition() < howMuch * clicksPerInch && rightIntake.getPosition() < howMuch * clicksPerInch) {

            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }


    private void moveArm(int armLevel, double speed) {
        // Initialize the servos for intake
        leftArm = hardwareMap.get(DcMotor.class, "LeftArm");
        rightArm = hardwareMap.get(DcMotor.class, "RightArm");

        // Set the direction of the motors
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        //set run mode
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the arm positions to 0
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);

        //This is just for now until we read the encoder values
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // fetch motor positions
        leftArmPos = leftArm.getCurrentPosition();
        rightArmPos = rightArm.getCurrentPosition();

        telemetry.addData("Arm Position", "Left Arm: %7d : Right Arm: %7d", leftArmPos, rightArmPos);

        //set the zero power behavior
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set the new positions
        int[] armPositions = {9 /*home*/, 39 /*Low*/, 58 /*Medium*/, 150 /*High*/};

        // set power to move arm
        leftArm.setPower(speed);
        rightArm.setPower(speed);

        // move arm to new position
        leftArm.setTargetPosition(armPositions[armLevel]);
        rightArm.setTargetPosition(armPositions[armLevel]);

        telemetry.addData("Motor Position Left arm and Right arm", "Left Arm: " + leftArm.getCurrentPosition() + " Right Arm: " + rightArm.getCurrentPosition());
        while (leftArm.getCurrentPosition() < armPositions[armLevel] && rightArm.getCurrentPosition() < armPositions[armLevel]) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        leftArm.setPower(0);
        rightArm.setPower(0);
    }


    public void strafe(int howMuch, double speed) {
        backleft = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        backright = hardwareMap.get(DcMotor.class, "BackRightWheel");
        frontleft = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        frontright = hardwareMap.get(DcMotor.class, "FrontRightWheel");
        // The right motors need reversing
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setTargetPosition(0);
        frontleft.setTargetPosition(0);
        backleft.setTargetPosition(0);
        backright.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // howMuch is in inches. A negative howMuch moves backward.

        // fetch motor positions
        lfPos = frontleft.getCurrentPosition();
        rfPos = frontright.getCurrentPosition();
        lrPos = backleft.getCurrentPosition();
        rrPos = backright.getCurrentPosition();

        // calculate new targets
        lfPos -= howMuch * clicksPerInch;
        rfPos += howMuch * clicksPerInch;
        lrPos += howMuch * clicksPerInch;
        rrPos -= howMuch * clicksPerInch;

        // move robot to new position
        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);

        frontleft.setTargetPosition(lrPos);
        frontright.setTargetPosition(rfPos);
        backleft.setTargetPosition(lrPos);
        backright.setTargetPosition(rrPos);


        while ( Math.abs(lfPos - frontleft.getCurrentPosition()) > tol
                || Math.abs(rfPos - frontright.getCurrentPosition()) > tol
                || Math.abs(lrPos - backleft.getCurrentPosition()) > tol
                || Math.abs(rrPos - backright.getCurrentPosition()) > tol) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
    }

    public void turnClockwise(int whatAngle, double speed) {
        backleft = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        backright = hardwareMap.get(DcMotor.class, "BackRightWheel");
        frontleft = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        frontright = hardwareMap.get(DcMotor.class, "FrontRightWheel");


        // The right motors need reversing
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        // whatAngle is in degrees. A negative whatAngle turns counterclockwise.
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setTargetPosition(0);
        frontleft.setTargetPosition(0);
        backleft.setTargetPosition(0);
        backright.setTargetPosition(0);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // fetch motor positions
        lfPos = frontleft.getCurrentPosition();
        rfPos = frontright.getCurrentPosition();
        lrPos = backleft.getCurrentPosition();
        rrPos = backright.getCurrentPosition();

        // calculate new targets
        lfPos += whatAngle * clicksPerDeg;
        rfPos -= whatAngle * clicksPerDeg;
        lrPos += whatAngle * clicksPerDeg;
        rrPos -= whatAngle * clicksPerDeg;

        // move robot to new position
        frontleft.setTargetPosition(lfPos);
        frontright.setTargetPosition(rfPos);
        backleft.setTargetPosition(lrPos);
        backright.setTargetPosition(rrPos);
        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);

        // wait for move to complete
        while (frontleft.isBusy() && frontright.isBusy() &&
                backleft.isBusy() && backright.isBusy()) {


            // Display it for the driver.
            telemetry.addLine("Turn Clockwise");
            telemetry.addData("Target", "%7d :%7d", lfPos, rfPos, lrPos, rrPos);
            telemetry.addData("Actual", "%7d :%7d", frontleft.getCurrentPosition(),
                    frontright.getCurrentPosition(), backleft.getCurrentPosition(),
                    backright.getCurrentPosition());
            telemetry.update();
        }
        try {
            Thread.sleep(5);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        // Stop all motion;
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);
    }

}
