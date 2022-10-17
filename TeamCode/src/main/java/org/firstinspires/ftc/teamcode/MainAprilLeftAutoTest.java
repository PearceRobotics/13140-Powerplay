/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */



    @Autonomous(name="Left Auto", group="Linear Opmode")  // @TeleOp(...) is the other common choice
// @Disabled
    public class MainAprilLeftAutoTest extends LinearOpMode {

        // April Tags decleration
        OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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



    // Declare Devices
        DcMotor frontleft = null;
        DcMotor frontright = null;
        DcMotor backleft = null;
        DcMotor backright = null;
        DcMotor rightArm = null;
        DcMotor leftArm = null;
        DcMotor intake = null;



        // drive motor position variables
        private int lfPos; private int rfPos; private int lrPos; private int rrPos;

        // operational constants
        private double fast = .75; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
        private double medium = 0.5; // medium speed
        private double slow = 0.30; // slow speed
        private double clicksPerInch = 44.56; // empirically measured 4x encoding
        private double clicksPerDeg = 9.02; // empirically measured 4x encoding
        private double tol = .1 * clicksPerInch;
        private double armPower = 1.0;
        int armPosition = 0;
        int[] armLevel = {0, 145, 470};
        //private double 45 = 90 * 9.45 - 570.6
        String target;

        @Override
        public void runOpMode() {
            telemetry.setAutoClear(true);

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


            // Initialize the hardware variables.
            backleft = hardwareMap.get(DcMotor.class, "BackLeftWheel");
            backright = hardwareMap.get(DcMotor.class, "BackRightWheel");
            frontleft = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
            frontright = hardwareMap.get(DcMotor.class, "FrontRightWheel");
            intake = hardwareMap.get(DcMotor.class, "Intake");
            leftArm = hardwareMap.get(DcMotor.class, "LeftArm");
            rightArm = hardwareMap.get(DcMotor.class, "RightArm");


            // The right motors need reversing
            frontright.setDirection(DcMotor.Direction.REVERSE);
            frontleft.setDirection(DcMotor.Direction.FORWARD);
            backright.setDirection(DcMotor.Direction.REVERSE);
            backleft.setDirection(DcMotor.Direction.FORWARD);

            // Set the drive motor run modes:
            frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontright.setTargetPosition(0);
            frontleft.setTargetPosition(0);
            backleft.setTargetPosition(0);
            backright.setTargetPosition(0);
            leftArm.setTargetPosition(0);
            rightArm.setTargetPosition(0);
            intake.setTargetPosition(0);

            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArm.setPower(1.0);
            rightArm.setPower(1.0);
            leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            telemetry.setMsTransmissionInterval(50);

            // April Tags initialization
            if (opModeIsActive()) {
                while (opModeIsActive()) {
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

                        if (detections.get(0).id == 1) {
                            telemetry.addLine("Target is Left Parking");
                            target = "left";
                            break;
                        } else if (detections.get(0).id == 2) {
                            telemetry.addLine("Target is Middle Parking");
                            target = "middle";
                            break;
                        } else {
                            telemetry.addLine("Target is Right Parking");
                            target = "right";
                            break;
                        }
                    }

                    sleep(20);
                }
            }



            // *****************Dead reckoning list*************
            // Distances in inches, angles in deg, speed 0.0 to 0.6
            //All moveforwards with number 5 need to be calculated still

            //Move the robot forward to top line of starting box
            moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
            //Turn robot 45 right
            turnClockwise(45, 0.75);
            //Move robot forward to the right side of the cone
            moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
            //Turn robot 90 left
            turnClockwise(-45, 0.75);
            //Move robot forward to the top of the cone
            moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
            //Turn robot +/-90 towards the high goal
            turnClockwise(90, 0.75);
            //Move robot forward to the high goal
            moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
            if  (frontleft.isBusy())
            {
                leftArm.setTargetPosition(armLevel[3]);
                rightArm.setTargetPosition(armLevel[3]);
            }
            //Outtake the cone
            intakePosition(20, 1.0);

            if (target == "left")
            {
                //Move robot backwards from the high goal
               moveForward(-20 /*Distance Needs to be re-calculated*/, 1.0);
               //Turn robot facing the left field wall
               turnClockwise(135, 0.75);
               //Move robot forward towards left field wall
               moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
               //Turn robot towards the left parking
               turnClockwise(90, 0.75);
               //Move towards the left parking
               moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
               leftArm.setTargetPosition(armLevel[0]);
               rightArm.setTargetPosition(armLevel[0]);
            }
            else if (target == "middle")
            {
                //Move robot backwards from the high goal
                moveForward(-20 /*Distance Needs to be re-calculated*/, 1.0);
                //Turn robot facing the cone
                turnClockwise(135, 0.75);
                //Move robot forward towards the middle parking
                moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
                leftArm.setTargetPosition(armLevel[0]);
                rightArm.setTargetPosition(armLevel[0]);
            }
            else
            {
                //Move robot backwards from the high goal
                moveForward(-20 /*Distance Needs to be re-calculated*/, 1.0);
                //Turn robot facing the right field wall
                turnClockwise(45, 0.75);
                //Move robot forward towards right field wall
                moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
                //Turn robot towards the right parking
                turnClockwise(90, 0.75);
                //Move towards the right parking
                moveForward(20 /*Distance Needs to be re-calculated*/, 1.0);
                leftArm.setTargetPosition(armLevel[0]);
                rightArm.setTargetPosition(armLevel[0]);
            }


        }

        private void moveForward(int howMuch, double speed) {
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

        private void moveRight(int howMuch, double speed) {
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

            intake.getCurrentPosition();
            intake.setTargetPosition((int) (howMuch * clicksPerInch));
            intake.setPower(speed);

            while (intake.getCurrentPosition() < howMuch * clicksPerInch ) {

                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

        }
         private void strafe(int howMuch, double speed) {
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

        private void turnClockwise(int whatAngle, double speed) {
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

