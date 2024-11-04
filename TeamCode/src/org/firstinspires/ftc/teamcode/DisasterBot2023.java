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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Disaster Bot 2023")
public class DisasterBot2023 extends LinearOpMode {
    private boolean flipDirection = false;
    private boolean yButtonDown = false;

    @Override
    public void runOpMode() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");

        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor"); // m2
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor"); // m3
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor"); // m1
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor"); // m4

        // needed only for the virtual bot
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for driver to press Play
        waitForStart();

        // Run until driver presses Stop
        while (opModeIsActive()) {

//            double leftStickY = gamepad1.left_stick_y;
//            double leftStickX = -gamepad1.left_stick_x;
//            double rightStickX = -gamepad1.right_stick_x;

            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = -gamepad1.left_stick_y;
            double rightStickX = -gamepad1.right_stick_x;

            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean yButton = gamepad1.y;

            if (yButton) {
                yButtonDown = true;
            }
            if (!yButton && yButtonDown) {
                yButtonDown = false;
                flipDirection = !flipDirection;
            }

            // Strafe right (or left if direction is flipped)
            if (rightBumper) {
                leftStickX = 0.5 * (flipDirection ? 1 : -1);
            }
            // Strafe right (or right if direction is flipped)
            if (leftBumper) {
                leftStickX = 0.5 * (flipDirection ? -1 : 1);
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            if (flipDirection) {
                leftStickX *= -1;
                leftStickY *= -1;
            }

            // add a dead zone to the left joystick strafe
            if (Math.abs(leftStickX) < 0.2){
                leftStickX = 0;
            }

            double denominator = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(rightStickX), 1);
            denominator = 1;
            double frontLeftPower = (leftStickY + leftStickX + rightStickX) / denominator;
            double frontRightPower = (leftStickY - leftStickX - rightStickX) / denominator;
            double backLeftPower = (leftStickY - leftStickX + rightStickX) / denominator;
            double backRightPower = (leftStickY + leftStickX - rightStickX) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Send telemetry message to signify robot running;
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("FL encoder", "%d", frontLeftMotor.getCurrentPosition());

            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("FR encoder", "%d", frontRightMotor.getCurrentPosition());

            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("BL encoder", "%d", backLeftMotor.getCurrentPosition());

            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.addData("BR encoder", "%d", backRightMotor.getCurrentPosition());

            telemetry.addData("Flipped direction?", "%b", flipDirection);

            telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);


            telemetry.update();
        }
    }
}
