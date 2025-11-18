/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Standard Drive", group="Linear OpMode")

public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;

    static final double SHOOTING_SPEED = 1.0;

    private DcMotor outtakeLeft = null;
    private DcMotor outtakeRight = null;

    private Servo door = null;
//    private Servo conveyorLeft = null;
//    private Servo conveyorRight = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables to the config in the control hub
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fL");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fR");
        backRightDrive = hardwareMap.get(DcMotor.class, "bR");
        intakeMotor = hardwareMap.get(DcMotor.class, "i");
        outtakeLeft = hardwareMap.get(DcMotor.class, "oL");
        outtakeRight = hardwareMap.get(DcMotor.class, "oR");
        door = hardwareMap.get(Servo.class, "d");
        //conveyorLeft = hardwareMap.get(Servo.class, "cL");
        //conveyorRight = hardwareMap.get(Servo.class, "cR");

        // setting direction for all DcMotors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);

        //run using encoder
        //outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //outtakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;
            
            //brakes
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }


            /* This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // END OF DRIVE CODE

                // Intake and Outtake Code
                if (gamepad1.left_bumper){
                    intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                    intakeMotor.setPower(1.0);
                }
                else if (gamepad1.right_bumper){
                    intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                    intakeMotor.setPower(1.0);
                }
                else{
                    intakeMotor.setPower(0);
                }

                //double outtakeLeft_Position = outtakeLeft.getCurrentPosition();
                //double outtakeRight_Position = outtakeRight.getCurrentPosition();
                if (gamepad1.left_trigger == 1.0) {
                    outtakeLeft.setPower(0.315);
                    outtakeRight.setPower(0.315);
                    runtime.reset();
                }
                if(gamepad1.right_trigger == 1.0){


                    while (opModeIsActive() && (runtime.seconds() < 0.1)) {
                        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                        intakeMotor.setPower(SHOOTING_SPEED);
                        telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    intakeMotor.setPower(0);
                    for (float i = 0.0F; i < 3; i++) {
                        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
                        intakeMotor.setPower(SHOOTING_SPEED);
                        runtime.reset();
                        while (opModeIsActive() && (runtime.seconds() < 0.1)) {
                            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                            telemetry.update();
                        }

                        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
                        intakeMotor.setPower(SHOOTING_SPEED);
                        runtime.reset();
                        while (opModeIsActive() && (runtime.seconds() < 0.2+i/7)) {
                            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                            telemetry.update();
                        }

                        intakeMotor.setPower(0);
                        runtime.reset();
                        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
                            telemetry.update();
                        }
                    }

                    /*
                    //outtakeLeft.setVelocity(75);
                    //outtakeRight.setVelocity(75);
                    outtakeLeft.setPower(0.315);
                    outtakeRight.setPower(0.315);
*/
                }else{
                    //outtakeLeft.setVelocity(0);
                    //outtakeRight.setVelocity(0);
                    outtakeLeft.setPower(0);
                    outtakeRight.setPower(0);
                }


                // Push servo code, continuous
//                if (gamepad1.left_bumper) {
//                    // shoot the ball
//                    push.setPosition(0.7);
//                    sleep(500);
//                    push.setPosition(0.5);
//                } else if (gamepad1.right_bumper) {
//                    while (gamepad1.right_bumper && opModeIsActive()) {
//                        push.setPosition(0.7);
//                    }
//                    push.setPosition(0.5);
//                }

                if (gamepad1.y) {
                    // door servo at position zero
                    door.setPosition(0.5);
                } else if (gamepad1.b) {
                    // door servo at position one
                    door.setPosition(0.8);
                }


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                //telemetry.addData("Outtake Left Encoder Ticks: ", outtakeLeft.getVelocity());
                //telemetry.addData("Outtake Left Position", outtakeLeft_Position);
                //telemetry.addData("Outtake Right Encoder Ticks: ", outtakeRight.getVelocity());
                //telemetry.addData("Outtake Right Position", outtakeRight_Position);


            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
                //add telemetry for intake motor?
                telemetry.addData("Status", "Running");
                telemetry.update();

        }
    }
}