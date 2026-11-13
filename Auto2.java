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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
@Autonomous

public class Auto2 extends OpMode {
    /* Declare OpMode members. */
    MyRobot robot;
 /* private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal; */
    
    private final int STATE_WAITING = 0;
    private final int STATE_SHOOTING = 1;
    private final int STATE_DRIVE_FORWARD = 2;
    private final int STATE_ROTATE = 3;
    private final int STATE_DRIVE_FORWARD_2 = 4;
    private final int STATE_DONE = 99;
    private int current_state = STATE_WAITING;
    private boolean isStateInitialized = false;
    ElapsedTime timer = new ElapsedTime();
    

    @Override
    public void init() {
        robot = new MyRobot();
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) {
            robot.setAlliance(Constants.kALLIANCE_BLUE);
        }
        else if (gamepad1.bWasPressed()) {
            robot.setAlliance(Constants.kALLIANCE_RED);
        }
        telemetryAlliance();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        timer.startTime();
        current_state = STATE_WAITING;
        robot.resetYaw();
        robot.resetMotorPosition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Current State:", current_state);
        telemetry.addData("Launch State:", robot.getLaunchStateName());
        telemetry.addData("Motors Target", "left (%d), right (%d)", 
        robot.leftDrive.getTargetPosition(), 
        robot.rightDrive.getTargetPosition());
         telemetry.addData("Motors Current", "left (%d), right (%d)", 
        robot.leftDrive.getCurrentPosition(), 
        robot.rightDrive.getCurrentPosition());       
        switch (current_state) {
            case STATE_WAITING: {
                if (robot.processAprilTag()) {
                    current_state = STATE_SHOOTING;
                }
                else if (timer.seconds() >= 20) {
                    current_state = STATE_DRIVE_FORWARD;
                }
                break;
            }
            case STATE_SHOOTING: {
                if (!isStateInitialized) {
                    robot.startLaunch();
                    isStateInitialized = true;
                    timer.reset();
                }
                if (timer.seconds() >= 1) {
                    robot.launch();
                    }
                if (robot.getLaunchState() == 3) {
                    current_state = STATE_DRIVE_FORWARD;
                    isStateInitialized = false;
                    robot.cancelLaunch();
                    robot.stopMotors();
                    timer.reset();
                }
                break;
            }
            case STATE_DRIVE_FORWARD: {
                if (robot.drive(0.5, 320, DistanceUnit.MM, 2)) {
                    current_state = STATE_ROTATE;
                }
                break;
            }
            case STATE_ROTATE: {
                //if (robot.rotate(0.5, -90, AngleUnit.DEGREES, 2)) {
                //    current_state = STATE_DONE;
                //}
                
                if (!isStateInitialized) {
                    // Target a 90-degree turn relative to current orientation
                    robot.setupTurnTargets(90);
                    isStateInitialized = true;
                }

                // Execute rotation, check heading error tolerance
                if (robot.hasReachedTargetAngle()) {
                    robot.stopMotors();
                    current_state = STATE_DRIVE_FORWARD_2;
                    isStateInitialized = false;
                    //robot.runUsingEncoders();
                    robot.resetMotorPosition();
                }
                break;
            }
            case STATE_DRIVE_FORWARD_2: {
                if (robot.drive(0.5, -1099, DistanceUnit.MM, 2)) {
                    current_state = STATE_DONE;
                }
                break;
            }
            case STATE_DONE: {
                robot.stopMotors();
                break;
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    public void telemetryAlliance() {
        if (robot.team_alliance == Constants.kALLIANCE_BLUE) {
            telemetry.addData("Alliance:", "Blue");    
        }
        else if (robot.team_alliance == Constants.kALLIANCE_RED) {
            telemetry.addData("Alliance:", "Red");    
        }
    }
}
