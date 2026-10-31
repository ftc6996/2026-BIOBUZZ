/*
Copyright 2026 FIRST Tech Challenge Team 6996

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC
 * match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is
 * made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller
 * and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add
 * this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this
 * OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name = "Brian Auto", group = "Brian")
public class kitbotautobrian extends OpMode {
    /* Declare OpMode members. */
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private IMU imu = null;
    private DigitalChannel ledRED;
    private DigitalChannel ledGREEN;;

    ElapsedTime timer = new ElapsedTime();
    public final int LAUNCHER_TARGET_VELOCITY = 1230;// 1250, 1230
    public final int LAUNCHER_MIN_VELOCITY = 1215;// 1200, 1215
    public final double TURN_SPEED = .6;

    /*
    1. Increase D_TURN_GAIN if it still overshoots: 
        If the robot still speeds past the mark, try raising it from 0.003 to 0.005. 
        This increases the electronic braking force.
    2. Decrease P_TURN_GAIN if it starts shaking violently: 
        If the robot vibrates rapidly right around the target line, the proportional push is too sharp. 
        Drop it from 0.025 down to 0.018.
    3. Decrease minTurnPower if it gets stuck close to target: 
        If the damping is so effective that the robot stops moving completely 1.5 degrees short of the target line, 
        drop minTurnPower down to 0.10 or 0.08 to give it just enough baseline voltage to cross the finish line. 
    */
    static final double P_DRIVE_GAIN = 0.03;
    static final double P_TURN_GAIN  = 0.025;   // Gain for keeping straight lines, Lowered slightly to reduce initial aggressive overshoot
    static final double D_TURN_GAIN  = 0.003;   // Gain for rotating on the spot, Damping factor. Higher values brake harder against momentum.
    static final double HEADING_THRESHOLD = 1.0; // Allowed angle error slack in degrees, Tightened threshold for cleaner alignment

    private double targetHeading = 0.0;
    private double lastHeadingError = 0.0;      // Track previous error to calculate velocity
    private ElapsedTime turnTimer = new ElapsedTime(); // Tracks exact loop cycle durations

    double leftPower;
    double rightPower;
    double intakePower;
    boolean setTimer = false;

    private final int STATE_WAITING = 0;
    private final int STATE_DRIVE = 1;
    private final int STATE_DONE = 2;
    private final int STATE_ROTATE = 3;
    private int current_state = STATE_WAITING;
    private boolean isStateInitialized = false;

    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "intake1");
        rightIntakeServo = hardwareMap.get(CRServo.class, "intake2");
        ledRED = hardwareMap.get(DigitalChannel.class, "red");
        ledGREEN = hardwareMap.get(DigitalChannel.class, "green");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
        ledRED.setMode(DigitalChannel.Mode.OUTPUT);
        ledGREEN.setMode(DigitalChannel.Mode.OUTPUT);

        // IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version:", 2);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        ledRED.setState(false);
        ledGREEN.setState(false);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        timer.startTime();
        current_state = STATE_WAITING;
        imu.resetYaw();
        targetHeading = getRawHeading();
    }

    private void updateTelemetry() {
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Target: ", targetHeading);

        telemetry.addData("time (sec): ", timer.seconds());
        if (current_state == STATE_WAITING) {
            telemetry.addData("state: ", "WAITING");

            ledRED.setState(true);
            ledGREEN.setState(true);
        } else if (current_state == STATE_DRIVE) {
            telemetry.addData("state: ", "DRIVE");
            ledRED.setState(false);
            ledGREEN.setState(true);
        } else if (current_state == STATE_ROTATE) {
            telemetry.addData("state: ", "ROTATE");
            ledRED.setState(false);
            ledGREEN.setState(true);
        } else if (current_state == STATE_DONE) {
            telemetry.addData("state: ", "DONE");
            ledRED.setState(true);
            ledGREEN.setState(false);
        } else {
            telemetry.addData("state: ", "UNKNOWN");
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        updateTelemetry();
        switch (current_state) {
            case STATE_WAITING: {
                if (timer.seconds() > 5) {
                    current_state = STATE_DRIVE;
                    timer.reset();
                }
                break;
            }
            case STATE_DRIVE: {
                if (timer.seconds() < 2) {
                    arcadeDrive(.5, 0);
                    setIntakePower(1);
                } else {
                    current_state = STATE_ROTATE;
                    targetHeading = 90;
                    imu.resetYaw();
                    timer.reset();
                }
                break;
            }
            case STATE_ROTATE: {
                if (!isStateInitialized) {
                    // Target a 90-degree turn relative to current orientation
                    setupTurnTargets(targetHeading);
                    isStateInitialized = true;
                }

                // Execute rotation, check heading error tolerance
                if (hasReachedTargetAngle()) {
                    stopMotors();
                    current_state = STATE_DONE;
                    isStateInitialized = false;
                }
                break;
            }
            case STATE_DONE: {
                // stop everything
                stopMotors();
                setIntakePower(0);
                break;
            }
            default: {
                break;
            }
        } // switch
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        stopMotors();
        ledRED.setState(false);
        ledGREEN.setState(false);
    }

    double normalizeAngle(double angle)
    {
        // Normalize target heading boundary range limits
        while (angle > 180)
            angle -= 360;
        while (angle <= -180)
            angle += 360;
        
        return angle;
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to motors
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    void setIntakePower(double pwr) {
        intake.setPower(pwr);
        leftIntakeServo.setPower(-pwr);
        rightIntakeServo.setPower(-pwr);
    }

    private void setupTurnTargets(double degreesToTurn) {
        // Encoders are not utilized for RUN_TO_POSITION in spot turning
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Update target trajectory orientation line
        targetHeading = getRawHeading() + degreesToTurn;
        // Normalize target heading boundary range limits
        targetHeading = normalizeAngle(targetHeading);
        // Reset tracking history for clean calculation cycles
        lastHeadingError = targetHeading - getRawHeading();
    }

    private boolean hasReachedTargetAngle() {
        double headingError = targetHeading - getRawHeading();
        double deltaTime = turnTimer.seconds();
        turnTimer.reset();

        headingError = normalizeAngle(headingError);
        // Stop checking if heading is within tolerance threshold limit limits
        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            return true;
        }

        // 1. Proportional Calculation (How far are we from target?)
        double pTerm = headingError * P_TURN_GAIN;

        // 2. Derivative Calculation (How fast are we closing in on target?)
        double errorChange = headingError - lastHeadingError;
        lastHeadingError = headingError; // Save history for next loop execution
        
        double dTerm = 0.0;
        if (deltaTime > 0) {
            // Error change over time gives us the rotational velocity tracking profile
            dTerm = (errorChange / deltaTime) * D_TURN_GAIN;
        }

        // Combine terms. Notice dTerm naturally works against pTerm when velocity is high.
        double turnPower = pTerm + dTerm;
        
        // Static friction threshold constraints
        double minTurnPower = 0.12; 

        if (turnPower > 0) {
            turnPower = Range.clip(turnPower, minTurnPower, TURN_SPEED);
        } else {
            turnPower = Range.clip(turnPower, -TURN_SPEED, -minTurnPower);
        }

        // Tank steering mechanics (Opposite forces to spin safely in place)
        // leftDrive.setPower(-turnPower);
        // rightDrive.setPower(turnPower);
        arcadeDrive(0, -turnPower);

        return false;
    }

    private void applySteeringCorrection(double speed) {
        double headingError = targetHeading - getRawHeading();
        headingError = normalizeAngle(headingError);

        double steerCorrection = headingError * P_DRIVE_GAIN;

        if (leftDrive.getTargetPosition() < leftDrive.getCurrentPosition()) {
            steerCorrection = -steerCorrection;
        }

        double leftPower = speed + steerCorrection;
        double rightPower = speed - steerCorrection;

        leftDrive.setPower(Range.clip(Math.abs(leftPower), -1.0, 1.0));
        rightDrive.setPower(Range.clip(Math.abs(rightPower), -1.0, 1.0));
    }

    private double getRawHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    private void stopMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
