package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.lang.Math;
import java.util.List;


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
    private PIDController controller;
    private boolean armPaused = false;

    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.1;
    private double speedmultiplier = 0.75;
    public static int target = 0;
    public boolean locking = false;
    public double lockpos = 0;
    public boolean prevx = false;

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
        } else if (gamepad1.left_trigger > 0.5) {
            leftPower  = Math.max(Math.min((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 4, 0.25), -0.25);
            rightPower = Math.max(Math.min((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 4, 0.25), -0.25);
        } else {
            leftPower  = Math.max(Math.min((-gamepad1.left_stick_y + gamepad1.right_stick_x) / 2, 0.5), -0.5);
            rightPower = Math.max(Math.min((-gamepad1.left_stick_y - gamepad1.right_stick_x) / 2, 0.5), -0.5);
        }
        if (gamepad2.right_trigger > 0.5) {
            speedmultiplier = 0.5;
        } else {
            speedmultiplier = 1;
        }
        armPower = -gamepad2.left_stick_y*0.85*speedmultiplier;
        wristUp = gamepad2.dpad_up;
        wristDown = gamepad2.dpad_down;
        clawButtonOpen = gamepad2.a;
        clawButtonClose = gamepad2.b;
        launchButton = gamepad2.right_bumper;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        if (clawButtonOpen) {
            claw.setPosition(0.55);
        }
        if (clawButtonClose) {
            claw.setPosition(0.67);
        }
        if (wristDown) {
            wrist.setPosition(0);
        } else if (wristUp) {
            wrist.setPosition(1.1);
        } else if (gamepad2.y) {
            wrist.setPosition(0.5);
        }
        telemetry.addData("Arm Power", armPower);
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
        if (gamepad2.x && !prevx) {
            locking = !locking;
            if (locking) {
                lockpos = leftArm.getCurrentPosition();
            }
        }
        prevx = gamepad2.x;
        if (locking) {
            rightArm.setTargetPosition((int) lockpos);
            leftArm.setTargetPosition((int) lockpos);
            rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftArm.setPower(1);
            rightArm.setPower(1);
        } else {
            rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightArm.setPower((armPower));
            leftArm.setPower((armPower));
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
}
