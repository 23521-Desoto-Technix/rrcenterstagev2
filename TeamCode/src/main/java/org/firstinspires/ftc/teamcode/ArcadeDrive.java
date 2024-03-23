package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.lang.Math;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Arcade Drive", group="Iterative Opmode")
@Config
public class ArcadeDrive extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftArm = null;
    private DcMotor rightArm = null;
    private Servo wrist = null;
    private Servo claw = null;
    private Servo launcher = null;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private double armPosition = 0;
    private PIDController controller;
    private boolean armPaused = false;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.1;
    private double speedmultiplier = 0.75;
    public static int target = 0;

    public double buttonPressToPower (boolean buttonPress) {
        double buttonPower = 0.0;
        if (buttonPress) {
            buttonPower = 90;
        }
        return buttonPower;
    }


    /*
     * Code to run ONCE when the driver hits INIT
     */
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
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        leftArm = hardwareMap.get(DcMotor.class, "left_arm");
        rightArm = hardwareMap.get(DcMotor.class, "right_arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        launcher = hardwareMap.get(Servo.class, "launcher");
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.setPosition(270);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(0.67);
        launcher.setDirection(Servo.Direction.REVERSE);
        launcher.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        final CameraStreamProcessor processor = new CameraStreamProcessor();
        initAprilTag(processor);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Running");
        leftArm.setTargetPosition(leftArm.getCurrentPosition());
        rightArm.setTargetPosition(rightArm.getCurrentPosition());
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double armPower;
        boolean wristUp;
        boolean wristDown;
        boolean clawButtonOpen;
        boolean clawButtonClose;
        boolean scoreButton;
        double x = claw.getPosition();
        double y = wrist.getPosition();
        boolean launchButton;
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        if (gamepad1.right_trigger > 0.5) {
            leftPower  = -gamepad1.left_stick_y + gamepad1.right_stick_x;
            rightPower = -gamepad1.left_stick_y - gamepad1.right_stick_x;
        } else {
            leftPower  = Math.max(Math.min((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 2, 0.5), -0.5);
            rightPower = Math.max(Math.min((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 2, 0.5), -0.5);
        }
        armPower = -gamepad2.left_stick_y*0.85;
        wristUp = gamepad2.dpad_up;
        wristDown = gamepad2.dpad_down;
        clawButtonOpen = gamepad2.a;
        clawButtonClose = gamepad2.b;
        scoreButton = gamepad2.x;
        launchButton = gamepad2.right_bumper;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        telemetryAprilTag();
        if (clawButtonOpen) {
            claw.setPosition(0.58);
        }
        if (clawButtonClose) {
            claw.setPosition(0.67);
        }
        if (wristDown) {
            wrist.setPosition(0);
        } else if (wristUp) {
            wrist.setPosition(1);
        } else if (gamepad2.y) {
            wrist.setPosition(0.5);
        }
        if (gamepad2.right_trigger > 0.5) {
            speedmultiplier = 0.4;
        } else {
            speedmultiplier = 0.75;
        }
        telemetry.addData("right trigger", gamepad2.right_trigger);
        telemetry.addData("speedmultiplier", speedmultiplier);
        /*
        if (armPower == 0){
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftArm.setPower(armPower);
            rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightArm.setPower(armPower);
            leftArm.setTargetPosition(leftArm.getCurrentPosition());
            rightArm.setTargetPosition(rightArm.getCurrentPosition());
        }
        armPosition += armPower * 1;
        telemetry.addData("Arm Position", armPosition);
        leftArm.setTargetPosition((int) armPosition);
        rightArm.setTargetPosition((int) armPosition);
        leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftArm.setPower(1);
        rightArm.setPower(1);*/
        int armPos = (leftArm.getCurrentPosition() + rightArm.getCurrentPosition()) / 2;
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos((double) armPos / 285 * Math.PI / 2) * f;
        double power = pid + ff;
        if (!armPaused && armPower == 0) {
            armPaused = true;
            target = armPos;
        } else if (armPower != 0) {
            armPaused = false;
        }
        target += (int) (armPower * 10);
        if (gamepad2.left_trigger > 0.5) {
            rightArm.setPower(-0.5);
            leftArm.setPower(-0.5);
        } else {
            rightArm.setPower((armPower + ff));
            leftArm.setPower((armPower + ff));
        }
        telemetry.addData("leftTrigger", gamepad2.left_trigger);
        telemetry.addData("armPos", armPos);
        telemetry.addData("power", power);
        telemetry.addData("ff", Math.cos((double) armPos / 1240 * Math.PI / 2));
        telemetry.addData("target", target);

        launcher.setPosition(buttonPressToPower(launchButton));
        controller.setPID(p, i, d);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Arm", leftArm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    private void initAprilTag(VisionProcessor processor) {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        builder.addProcessor(processor);
        builder.setCameraResolution(new Size(1280, 720));
        FtcDashboard.getInstance().startCameraStream((CameraStreamSource) processor, 0);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

    }
}
