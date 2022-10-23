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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name = "MainTeleop", group = "Robot code")

public class MainOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleft = null;
    private DcMotor backright = null;
    private DcMotor frontright = null;
    private DcMotor frontleft = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private DcMotor intake = null;


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
        leftArm = hardwareMap.get(DcMotor.class, "LeftArm");
        rightArm = hardwareMap.get(DcMotor.class, "RightArm");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        //magnet = hardwareMap.get(TouchSensor.class, "Magnet");

        // Robot wheel motor set Up ------------------------------
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);

        // Robot Arm motor set Up ------------------------------
//        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftArm.setTargetPosition(0);
//        rightArm.setTargetPosition(0);

        //telemetry.addData("Motor Position Left arm and Right arm", "Left Arm: " + leftArm.getCurrentPosition() + " Right Arm: " + rightArm.getCurrentPosition());
        //This is just for now until we read the encoder values
//        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftArm.setPower(1.0);
//        rightArm.setPower(1.0);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double leftArmPower = 1.0;
        double rightArmPower = 1.0;
        int armPosition = 0;
        int[] armLevel = {9 /*home*/, 39 /*Low*/, 58 /*Medium*/, 150 /*High*/};

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontleftpower = (y - x + rx) / denominator;
            double backleftpower = (y + x + rx) / denominator;
            double frontrightpower = (y + x - rx) / denominator;
            double backrightpower = (y - x - rx) / denominator;

            // Send calculated power to wheels
            backleft.setPower(backleftpower);
            backright.setPower(backrightpower);
            frontright.setPower(frontrightpower);
            frontleft.setPower(frontleftpower);

            // Code for mechanism to go
            // arm automation set up goes here
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

//            if (gamepad1.right_bumper){
//                leftArm.setPower(1.0);
//                rightArm.setPower(1.0);}
//            else if (gamepad1.left_bumper) {
//                leftArm.setPower(-1.0);
//            rightArm.setPower(-1.0);}
//            else{
//                leftArm.setPower(0);
//                rightArm.setPower(0);
//            }


            if (gamepad1.right_bumper)
                intake.setPower(-1.0);
            else if (gamepad1.left_bumper)
                intake.setPower(1.0);

            else
                intake.setPower(0.0);


            // Show the elapsed game time and wheel power, also arm position.
            telemetry.addData("Motor position", String.valueOf(frontleft.getCurrentPosition()), frontright.getCurrentPosition(), backleft.getCurrentPosition(), backright.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Arm Power", "left (%.2f), right (%.2f)", leftArm.getPower(), rightArm.getPower());
            telemetry.addData("Left arm value", "Left Arm Position: " + leftArm.getCurrentPosition());
            telemetry.addData("Right arm value", "Right Arm Position: " + rightArm.getCurrentPosition());
            telemetry.addData("Motors", "frontleft (%.2f), frontright (%.2f), backleft (%.2f), backright (%.2f)", frontleftpower, frontrightpower, backleftpower, backrightpower);
            telemetry.update();
        }
    }
}

