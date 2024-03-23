package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class armpos extends LinearOpMode {
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    public void runOpMode() {
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        leftArm.setDirection(DcMotor.Direction.REVERSE);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Encodey", leftArm.getCurrentPosition());
            telemetry.update();
        }
    }
}
