package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="Red Backstage")
public class Red extends LinearOpMode {
    private FirstPipelineRevised firstPipelineRevised; //Create an object of the VisionProcessor Class
    private VisionPortal portal;
    private Servo claw = null;
    IMU imu;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private double delayCount = 0;

    public static class CameraStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
            // do nothing
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        final CameraStreamProcessor processor = new CameraStreamProcessor();
        firstPipelineRevised = new FirstPipelineRevised();
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setTargetPosition(0);
        rightArm.setTargetPosition(0);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(firstPipelineRevised)
                .addProcessor(processor)
                .setCameraResolution(new Size(1280, 720))
                .build();

        FtcDashboard.getInstance().startCameraStream(processor, 0);
        claw = hardwareMap.get(Servo.class, "claw");
        Pose2d beginPose = new Pose2d(0, 0, 0);
        TankDrive drive = new TankDrive(hardwareMap, beginPose);
        ModernRoboticsI2cRangeSensor rangeSensor;
        boolean do_yellow = true;
        boolean park = false;
        boolean lastup = false;
        boolean lastdown = false;
        claw.setPosition(0.7);
        while (opModeInInit()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.addData("Yellow? ", do_yellow);
            if (park) {
                telemetry.addLine("Park: middle");
            } else {
                telemetry.addLine("Park: corner");
            }
            telemetry.addData(String.valueOf(delayCount), "Delay");
            telemetry.update();
            if (gamepad1.dpad_right) {
                do_yellow = true;
            }
            if (gamepad1.dpad_left) {
                do_yellow = false;
            }
            if (gamepad1.x) {
                park = true;
            }
            if (gamepad1.b) {
                park = false;
            }
            if (gamepad1.dpad_up && !lastup) {
                delayCount += 1;
            }
            if (gamepad1.dpad_down && !lastdown) {
                delayCount -= 1;
            }
            lastup = gamepad1.dpad_up;
            lastdown = gamepad1.dpad_down;
            if (delayCount < 0) {
                delayCount = 0;
            }
        }
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addLine(String.valueOf(firstPipelineRevised.getSelection()));
            telemetry.update();
            double selection = firstPipelineRevised.getSelection();
            sleep((long) (1000 * delayCount));
            if (selection == 1) {
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(12.5, 0), 0)
                                .splineTo(new Vector2d(27, 5.5), Math.toRadians(45))
                                .setReversed(true)
                                .splineTo(new Vector2d(34.5, -33.5), Math.toRadians(-90))
                                .build());
                bomb();
                if (park) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(51, -30), Math.toRadians(90))
                                    .setReversed(true)
                                    .lineToY(-50)
                                    .build());
                } else {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(Math.toRadians(-90))
                                    .setReversed(true)
                                    .lineToX(0)
                                    .build());
                }
            } else if (selection == 2) {
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(30, 0), Math.toRadians(0))
                                .setReversed(true)
                                .splineTo(new Vector2d(27, -34.5), Math.toRadians(-90))
                                .build());
                bomb();
                if (park) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(51, -30), Math.toRadians(90))
                                    .setReversed(true)
                                    .lineToY(-50)
                                    .build());
                } else {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(Math.toRadians(-90))
                                    .setReversed(true)
                                    .lineToX(0)
                                    .build());
                }
            } else if (selection == 3) {
                Actions.runBlocking(
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(12.5, 0), 0)
                                .splineTo(new Vector2d(27, -5.5), Math.toRadians(-45))
                                .setReversed(true)
                                .splineTo(new Vector2d(16.5, -34.5), Math.toRadians(-90))
                                .build());
                bomb();
                if (park) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(51, -30), Math.toRadians(90))
                                    .setReversed(true)
                                    .lineToY(-50)
                                    .build());
                } else {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(Math.toRadians(-90))
                                    .setReversed(true)
                                    .lineToX(0)
                                    .build());
                }
            }
            stop();
        }

    }

    public void bomb() {
        leftArm.setPower(1);
        rightArm.setPower(1);
        while ((leftArm.getCurrentPosition() < 1100) && !isStopRequested()) {
        }
        leftArm.setPower(0.5);
        rightArm.setPower(0.5);
        while ((leftArm.getCurrentPosition() < 2100) && !isStopRequested()) {
        }
        sleep(200);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftArm.setPower(0);
        rightArm.setPower(0);
        sleep(200);
        claw.setPosition(0.5);
        sleep(200);
        leftArm.setPower(-1);
        rightArm.setPower(-1);
        sleep(750);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftArm.setPower(0);
        rightArm.setPower(0);
    }
}