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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Teleop", group = "Robot code")

public class MainOpMode extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontright = null;
    private DcMotor frontleft = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private Servo leftIntake = null;
    private Servo rightIntake = null;
    private TouchSensor magnet = null;
    int armPosition = 0;
    int[] armLevel = {9 /*home*/, 39 /*Low*/, 58 /*Medium*/, 100 /*High*/};


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        backleft = hardwareMap.get(DcMotor.class, "BackLeftWheel");
        backright = hardwareMap.get(DcMotor.class, "BackRightWheel");
        frontleft = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
        frontright = hardwareMap.get(DcMotor.class, "FrontRightWheel");

        // Initialize the arm and intake motors
        leftArm = hardwareMap.get(DcMotor.class, "LeftArm");
        rightArm = hardwareMap.get(DcMotor.class, "RightArm");



        // Initialize the servos for intake
        leftIntake = hardwareMap.get(Servo.class, "Intake");
        rightIntake = hardwareMap.get(Servo.class, "Intake2");

        // Initialize the sensors
        magnet = hardwareMap.get(TouchSensor.class, "Magnet");

        // Robot wheel motor set Up ------------------------------
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backright.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);

     //    Robot Arm motor set Up ------------------------------
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
//        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftArm.setTargetPosition(0);
//        rightArm.setTargetPosition(0);

        //This is just for now until we read the encoder values
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftArm.setPower(1.0);
//        rightArm.setPower(1.0);
//
//        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Robot Intake servo set Up ------------------------------
        leftIntake.setDirection(Servo.Direction.FORWARD);
        rightIntake.setDirection(Servo.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            // Send calculated power to wheels
            backleft.setPower(v3);
            backright.setPower(v4);
            frontright.setPower(v2);
            frontleft.setPower(v1);


//            if (magnet.isPressed()) {
//                telemetry.addData("Magnet", "Arm is at home");
//                leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                leftArm.setTargetPosition(0);
//                rightArm.setTargetPosition(0);
//                //This is just for now until we read the encoder values
//                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else {
//                telemetry.addData("Magnet", "Arm is not at home");
//            }

            // Arm control
//            // arm automation set up goes here
//            if (gamepad1.x) {
//                armPosition = armLevel[0];
//            } else if (gamepad1.a) {
//                armPosition = armLevel[1];
//            } else if (gamepad1.b) {
//                armPosition = armLevel[2];
//            } else if (gamepad1.y) {
//                armPosition = armLevel[3];
//            }
//            leftArm.setTargetPosition(armPosition);
//            rightArm.setTargetPosition(armPosition);


            if(gamepad1.left_trigger > 0.01){
                leftArm.setPower(0.75);
                rightArm.setPower(0.75);
            }
            else if (gamepad1.right_trigger > 0.01){
                leftArm.setPower(-0.75);
                rightArm.setPower(-0.75);
            }
            else{
                leftArm.setPower(0.0);
                rightArm.setPower(0.0);
            }


            // Intake controls
            if (gamepad1.right_bumper)
            {
                leftIntake.setPosition(-5);
                rightIntake.setPosition(-5);
            }
            else if (gamepad1.left_bumper)
            {
                leftIntake.setPosition(5);
                rightIntake.setPosition(5);
            }
            else {
                leftIntake.getController().pwmDisable();
                rightIntake.getController().pwmDisable();
            }


            // Show the elapsed game time and wheel power, also arm position.
            telemetry.addData("Motor position", String.valueOf(frontleft.getCurrentPosition()), frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "frontleft (%.2f), frontright (%.2f), backleft (%.2f), backright (%.2f)", frontleft.getPower(), frontright.getPower(), backleft.getPower(), backright.getPower());
            telemetry.addData("Motor Position Left arm", "Left Arm: " + leftArm.getCurrentPosition());
            telemetry.addData("Motor Position Right arm", "Right Arm: " + rightArm.getCurrentPosition());
            telemetry.addData("Servo Position", "Left Intake: " + leftIntake.getPosition());
            telemetry.addData("Power", "rightpower: " + rightArm.getPower() + " leftpower: " + leftArm.getPower());
            telemetry.update();
        }
    }
}

