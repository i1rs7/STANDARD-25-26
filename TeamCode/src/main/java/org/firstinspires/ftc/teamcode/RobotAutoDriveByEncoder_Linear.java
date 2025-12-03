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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Encoder Auto Red", group="Robot")

public class RobotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backRightDrive = null;
    private DcMotorEx intakeMotor = null;

    private DcMotorEx outtakeLeft = null;
    private DcMotorEx outtakeRight = null;

    //drive counts per revolution conversion to distance
    static final double     DRIVE_COUNTS_PER_MOTOR_REV    = 384.5 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     DRIVE_WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     DRIVE_COUNTS_PER_INCH         = (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (DRIVE_WHEEL_DIAMETER_INCHES * 3.1415);

    //intake counts per revolution conversion to distance
    static final double     INTAKE_COUNTS_PER_MOTOR_REV    = 384.5 ; //
    static final double     INTAKE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     INTAKE_WHEEL_DIAMETER_INCHES   = 1.88976 ;     // For figuring circumference
    static final double     INTAKE_COUNTS_PER_INCH         = (INTAKE_COUNTS_PER_MOTOR_REV * INTAKE_GEAR_REDUCTION) /
                                                        (INTAKE_WHEEL_DIAMETER_INCHES * 3.1415);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     INTAKE_SPEED = 1.0;
    static final double target_RPM_close = 725;
    static final double target_RPM_far = 975;
    static final double target_range = 25;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "fR");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "bR");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "i");
        outtakeLeft = hardwareMap.get(DcMotorEx.class, "oL");
        outtakeRight = hardwareMap.get(DcMotorEx.class, "oR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // intake up
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //brakes
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                backLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition(),
                backRightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        intakeByEncoder(INTAKE_SPEED,10,5.0); // Intake balls 10 inches with 5 sec timeout
        //encoderDrive(DRIVE_SPEED,  75,  75, 5.0);  // S1: Backward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   24, -24, 4.0);  // S3: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -48, -48, 4.0);  // S4: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * DRIVE_COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_INCH);
            newBackRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * DRIVE_COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);


            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() && backLeftDrive.isBusy()
                   && frontRightDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,
                        newBackLeftTarget, newFrontRightTarget, newBackRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(),
                        frontRightDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    public void intakeByEncoder(double speed, double Inches, double timeoutS){
        int newIntakeTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newIntakeTarget = intakeMotor.getCurrentPosition() + (int) (Inches * INTAKE_COUNTS_PER_INCH);
            intakeMotor.setTargetPosition(newIntakeTarget);

            // Turn On RUN_TO_POSITION
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            intakeMotor.setPower(Math.abs(speed));

            //telemetry
            while (opModeIsActive() && intakeMotor.isBusy()) {
                telemetry.addData("Running to", " %.7f", Inches);
                telemetry.addData("New Intake Target", newIntakeTarget);
                telemetry.update();
            }

            //Stop all motion:
            intakeMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);   // optional pause after each move.
        }
    }
}
