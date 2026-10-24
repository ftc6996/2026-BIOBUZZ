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
 * This file contains a minimal example of an iterative (Non-Linear) "OpMode". An OpMode is a
 * 'program' that runs in either the autonomous or the TeleOp period of an FTC match. The names
 * of OpModes appear on the menu of the FTC Driver Station. When an selection is made from the
 * menu, the corresponding OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
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
    public final int LAUNCHER_TARGET_VELOCITY = 1230;//1250, 1230
    public final int LAUNCHER_MIN_VELOCITY = 1215;//1200, 1215
    public final double TURN_SPEED = .3;
    
    static final double P_DRIVE_GAIN = 0.03;    // Gain for keeping straight lines
    static final double P_TURN_GAIN  = 0.04;    // Gain for rotating on the spot
    static final double HEADING_THRESHOLD = 2.0; // Allowed angle error slack in degrees
    private double targetHeading = 0.0;         


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
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version:", 1);

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

    private void updateTelemetry()
    {
        telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("Target: ", targetHeading);
        
        telemetry.addData("time (sec): ",timer.seconds());
        if (current_state == STATE_WAITING)
        {
            telemetry.addData("state: ", "WAITING");
            
            ledRED.setState(true);
            ledGREEN.setState(true);
        }
        else if (current_state == STATE_DRIVE)
        {
            telemetry.addData("state: ", "DRIVE");
            ledRED.setState(false);
            ledGREEN.setState(true);
        }
        else if (current_state == STATE_ROTATE)
        {
            telemetry.addData("state: ", "ROTATE");
            ledRED.setState(false);
            ledGREEN.setState(true);
        }
        else if (current_state == STATE_DONE)
        {
            telemetry.addData("state: ", "DONE");
            ledRED.setState(true);
            ledGREEN.setState(false);
        }
        else
        {
            telemetry.addData("state: ", "UNKNOWN");
        }
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        updateTelemetry();
        switch(current_state)
        {
            case STATE_WAITING:
            {
                if (timer.seconds() > 5)
                {
                    current_state = STATE_DRIVE;
                    timer.reset();
                }
                break;
            }
            case STATE_DRIVE:
            {
                if (timer.seconds() < 2)
                {
                    arcadeDrive(.5,0);
                    setIntakePower(1);
                }
                else
                {
                    current_state = STATE_ROTATE;
                    targetHeading = 90;
                    imu.resetYaw();
                    timer.reset();
                }
                break;
            }
            case STATE_ROTATE:
            {
                if (!isStateInitialized) {
                    // Target a 90-degree turn relative to current orientation
                    targetHeading -= 10; //offset error
                    setupTurnTargets(targetHeading);
                    isStateInitialized = true;
                }
                
                if (isStateInitialized)
                {
                // Execute rotation, check heading error tolerance
                if (hasReachedTargetAngle()) {
                    stopMotors();
                    current_state = STATE_DONE;
                    isStateInitialized = false;
                }
                }
                break;
            }
            case STATE_DONE:
            {
                //stop everything
                arcadeDrive(0,0);
                setIntakePower(0);
                break;
            }
            default:
            {
                break;
            }
        } //switch
        
        /*
        //wait 5 secs
        if (timer.seconds() < 5)
        {
            //nothing
        }
        else 
        {
            
            if (timer.seconds() < 7)
            {
                current_state = STATE_DRIVE;
                //drive for 2 seconds
                arcadeDrive(-.5,0);
                setIntakePower(1);
            }
            else
            {
                current_state = STATE_DONE;
                //stop
                arcadeDrive(0,0);
                setIntakePower(0);
            }
            
        }
        
        */
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
    
    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to motors
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
    
    void setIntakePower(double pwr)
    {
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
        while (targetHeading > 180)  targetHeading -= 360;
        while (targetHeading <= -180) targetHeading += 360;
    }
    
    private boolean hasReachedTargetAngle() {
        double headingError = targetHeading - getRawHeading();

        // Normalize heading error wrap-arounds
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Stop checking if heading is within tolerance threshold limit limits
        if (Math.abs(headingError) <= HEADING_THRESHOLD) {
            return true; 
        }

        // Proportional power loop calculations. Minimum power floors are set to defeat static friction.
        double turnPower = headingError * P_TURN_GAIN;
        double minTurnPower = 0.15; 

        if (turnPower > 0) {
            turnPower = Range.clip(turnPower, minTurnPower, TURN_SPEED);
        } else {
            turnPower = Range.clip(turnPower, -TURN_SPEED, -minTurnPower);
        }

        // Tank steering mechanics (Opposite forces to spin safely in place)
        //leftDrive.setPower(-turnPower);
        //rightDrive.setPower(turnPower);
        arcadeDrive(0, -turnPower);

        return false;
    }
    
    private void applySteeringCorrection(double speed) {
        double headingError = targetHeading - getRawHeading();
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        double steerCorrection = headingError * P_DRIVE_GAIN;

        if (leftDrive.getTargetPosition() < leftDrive.getCurrentPosition()) {
            steerCorrection = -steerCorrection;
        }

        double leftPower  = speed + steerCorrection;
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
