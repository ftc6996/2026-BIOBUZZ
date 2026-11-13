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
import com.qualcomm.robotcore.robot.Robot;
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
@TeleOp

public class DriverController extends OpMode {
    /* Declare OpMode members. */
    MyRobot robot;
    int alliance = Constants.kNOT_SET;


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
        telemetry.addData("Version", "2");
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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetryAlliance();
        telemetryHeading();
        telemetryLaunch();
        telemetry.addData("Joystick", "left (%.2f), right (%.2f)", 
        gamepad1.left_stick_y, 
        gamepad1.right_stick_x);
        
        if (gamepad1.optionsWasPressed()) {
            robot.resetYaw();
        }
        if (gamepad1.left_bumper) {
            robot.startLaunch();
        }
        if (gamepad1.right_bumper) {
            robot.cancelLaunch();
        }
        
        robot.launch();
        robot.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x);
        robot.intake(gamepad1.right_trigger - gamepad1.left_trigger); 
        
        
        
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
    public void telemetryHeading() {
        double heading = robot.getRawHeading();
        telemetry.addData("Heading:", heading);
    }
    public void telemetryLaunch() {
        telemetry.addData("Launch State:", robot.getLaunchStateName());
    }
}
