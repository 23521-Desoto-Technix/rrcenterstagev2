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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class Arm extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0.005;

    public static int target = 0;

    private final double ticks_in_degree =  ((288.0 / 360.0) * 0.24);

    private DcMotorEx arm_motor_left;
    private DcMotorEx arm_motor_right;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor_left = hardwareMap.get(DcMotorEx.class, "left_arm");
        arm_motor_right = hardwareMap.get(DcMotorEx.class, "right_arm");
        arm_motor_left.setDirection(DcMotorEx.Direction.REVERSE);
        arm_motor_right.setDirection(DcMotorEx.Direction.FORWARD);
        arm_motor_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_motor_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = (arm_motor_left.getCurrentPosition() + arm_motor_right.getCurrentPosition()) / 2;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos((double) armPos / 285 * Math.PI / 2) * f;
        double power = pid + ff;

        arm_motor_right.setPower(power);
        arm_motor_left.setPower(power);
        telemetry.addData("armPos", armPos);
        telemetry.addData("power", power);
        telemetry.addData("ff", Math.cos((double) armPos / 285 * Math.PI / 2));
        telemetry.update();
    }
}