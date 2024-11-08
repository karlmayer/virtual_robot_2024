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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Disaster Bot 2024")
public class DisasterBot2024 extends LinearOpMode {
    private static final double LEFT_STICK_Y_DEAD_ZONE = 0.1;
    private static final double LEFT_STICK_X_DEAD_ZONE = 0.1;
    private static final double RIGHT_STICK_X_DEAD_ZONE = 0.1;
    private static final double RIGHT_STICK_Y_DEAD_ZONE = 0.1;
    private static final double SLOW_FACTOR = .5; // halve the speed
    private static final int ARM_MOTOR_LIMIT = 5000;
    private static final int SLIDE_MOTOR_LIMIT = 5000;

    // Adjust power for a defined dead zone
    private double adjustPower(double power, double deadZone) {
        double adjustedPower = 0;
        if (Math.abs(power) > deadZone) {
            adjustedPower = (power - Math.signum(power) * deadZone) / (1 - deadZone);
        }
        return adjustedPower;
    }

    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        // needed only for the virtual bot
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for driver to press Play
        waitForStart();

        // Run until driver presses Stop
        boolean yButtonDown = false;
        boolean slowMode = false;

        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x; // turn
            double leftStickY = -gamepad1.left_stick_y; // arm
            double rightStickX = gamepad1.right_stick_x; // strafe
            double rightStickY = -gamepad1.right_stick_y; // slide
            double rightTrigger = gamepad1.right_trigger; // forward
            double leftTrigger = gamepad1.left_trigger; // backward (both to brake)
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean dpad_up = gamepad1.dpad_up;
            boolean dpad_down = gamepad1.dpad_down;
            boolean aButton = gamepad1.a; // collection in
            boolean bButton = gamepad1.b; // collection out
            boolean yButton = gamepad1.y; // slow mode
            boolean xButton = gamepad1.x;

            // slow mode on & off
            if (yButton) {
                yButtonDown = true;
            }
            if (!yButton && yButtonDown) {
                yButtonDown = false;
                slowMode = !slowMode;
            }

            // driving control
            double forwardPower = rightTrigger - leftTrigger;
            if (rightTrigger > 0 && leftTrigger > 0) {
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (forwardPower < 0) {
                    forwardPower = 0;
                }
            } else {
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            double turnPower = -adjustPower(leftStickX, LEFT_STICK_X_DEAD_ZONE);
            double strafePower = adjustPower(rightStickX, RIGHT_STICK_X_DEAD_ZONE);
            double denominator = Math.max(Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
            if (slowMode) {
                denominator /= SLOW_FACTOR;
            }
            double frontLeftPower = (forwardPower + strafePower + turnPower) / denominator;
            double frontRightPower = (forwardPower - strafePower - turnPower) / denominator;
            double backLeftPower = (forwardPower - strafePower + turnPower) / denominator;
            double backRightPower = (forwardPower + strafePower - turnPower) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Slow Mode: ", "%b", slowMode);

//            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);

            telemetry.update();
        }
    }
}
