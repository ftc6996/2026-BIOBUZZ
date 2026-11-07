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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
import java.util.List;
import org.firstinspires.ftc.teamcode.Constants;

public class MyRobot {
    
    // Declare OpMode members.
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    private DcMotorEx launcher = null;
    private CRServo windmillServo = null;
    
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    
    
    private IMU imu = null;
    private YawPitchRollAngles robotOrientation;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private HardwareMap hardwareMap;
    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    
    public final int LAUNCHER_TARGET_VELOCITY = 1230;//1250, 1230
    public final int LAUNCHER_MIN_VELOCITY = 1215;//1200, 1215
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 402;
    final double WINDMILL_INTAKE_SPEED = -1.0; 
    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime turnTimer = new ElapsedTime();
    private double targetHeading = 0.0;
    private double lastHeadingError = 0.0;      // Track previous error to calculate velocity

    /*
     * These two variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the arcadeDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    private double leftPower;
    private double rightPower;

    // Create a variable to set to the intake.
    double intakePower;

    private static final boolean USE_WEBCAM = true;

    public int team_alliance = Constants.kNOT_SET;

    //Launch state machine
    private final int STATE_WAITING = 0;
    private final int STATE_START = 1;
    private final int STATE_LAUNCH = 2;
    private final int STATE_DONE = 3;
    private int launchState = STATE_WAITING;
    private ElapsedTime launchTimer = new ElapsedTime(); // Tracks exact loop cycle durations


    public void resetYaw()
    {
        imu.resetYaw();
    }
    public void arcadeDrive(double forward, double rotate)
    {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        /*
         * Send calculated power to motors
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void stop()
    {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    
    public int getLaunchState()
    {
        return launchState;
    }
    public String getLaunchStateName()
    {
        String state = "UNKNOWN";
        switch(launchState)
        {
            case STATE_WAITING:
            {
                state = "WAITING";
                break;
            }
            case STATE_START:
            {
                state = "START";
                break;
            }
            case STATE_LAUNCH:
            {
                state = "LAUNCHING";
                break;
            }
            case STATE_DONE:
            {
                state = "FINISHED";
                break;
            }
        }
        return state;
    }
    
    public void startLaunch()
    {
        if (launchState == STATE_WAITING)
        {
            launchState = STATE_START;
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
    }
    public void cancelLaunch()
    {
        launchState = STATE_DONE;
    }
    public void launch()
    {
        switch (launchState) {
            case STATE_START:
            {
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY)
                {
                    launchState = STATE_LAUNCH;
                    intake(.5);
                    windmillServo.setPower(1);
                    launchTimer.reset();
                }
                break;
            }
            case STATE_LAUNCH:
            {
                if (launchTimer.seconds() > 2)
                {
                    launchState = STATE_DONE;
                }
                break;
            }
            case STATE_DONE:
            {
                launchState = STATE_WAITING;
                launcher.setVelocity(0);
                windmillServo.setPower(0);
                intake(0);
                break;
            }
            default:
                break;
        }
    }

    public void intake(double pwr)
    {
        intakePower = pwr;
        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */
        intake.setPower(intakePower);
        leftIntakeServo.setPower(-intakePower);
        rightIntakeServo.setPower(-intakePower);
    }

    public void resetMotorPosition()
    {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
/**
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK implements
         * which allows us to accept different input units depending on the user's preference.
         * To use these, put both a double and a DistanceUnit as parameters in a function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so that's the
         * unit we request from distanceUnit. But if we want to use inches in our function, we could
         * use distanceUnit.toInches() instead!
         */
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) targetPosition);
        rightDrive.setTargetPosition((int) targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        /*
         * Here we check if we are within tolerance of our target position or not. We calculate the
         * absolute error (distance from our setpoint regardless of if it is positive or negative)
         * and compare that to our tolerance. If we have not reached our target yet, then we reset
         * the driveTimer. Only after we reach the target can the timer count higher than our
         * holdSeconds variable.
         */
        if((Math.abs(targetPosition - leftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) ||
        (Math.abs(targetPosition - rightDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)))
        {
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
    
   public void setupTurnTargets(double degreesToTurn) {
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

    public boolean hasReachedTargetAngle() {
        double headingError = targetHeading - getRawHeading();
        double deltaTime = turnTimer.seconds();
        turnTimer.reset();

        headingError = normalizeAngle(headingError);
        // Stop checking if heading is within tolerance threshold limit limits
        if (Math.abs(headingError) <= Constants.HEADING_THRESHOLD) {
            return true;
        }

        // 1. Proportional Calculation (How far are we from target?)
        double pTerm = headingError * Constants.P_TURN_GAIN;

        // 2. Derivative Calculation (How fast are we closing in on target?)
        double errorChange = headingError - lastHeadingError;
        lastHeadingError = headingError; // Save history for next loop execution
        
        double dTerm = 0.0;
        if (deltaTime > 0) {
            // Error change over time gives us the rotational velocity tracking profile
            dTerm = (errorChange / deltaTime) * Constants.D_TURN_GAIN;
        }

        // Combine terms. Notice dTerm naturally works against pTerm when velocity is high.
        double turnPower = pTerm + dTerm;
        
        // Static friction threshold constraints
        double minTurnPower = 0.12; 

        if (turnPower > 0) {
            turnPower = Range.clip(turnPower, minTurnPower, Constants.kAUTO_TURN_SPEED);
        } else {
            turnPower = Range.clip(turnPower, -Constants.kAUTO_TURN_SPEED, -minTurnPower);
        }

        // Tank steering mechanics (Opposite forces to spin safely in place)
        // leftDrive.setPower(-turnPower);
        // rightDrive.setPower(turnPower);
        arcadeDrive(0, -turnPower);

        return false;
    }

    public void applySteeringCorrection(double speed) {
        double headingError = targetHeading - getRawHeading();
        headingError = normalizeAngle(headingError);

        double steerCorrection = headingError * Constants.P_DRIVE_GAIN;

        if (leftDrive.getTargetPosition() < leftDrive.getCurrentPosition()) {
            steerCorrection = -steerCorrection;
        }

        double leftPower = speed + steerCorrection;
        double rightPower = speed - steerCorrection;

        leftDrive.setPower(Range.clip(Math.abs(leftPower), -1.0, 1.0));
        rightDrive.setPower(Range.clip(Math.abs(rightPower), -1.0, 1.0));
    }
    
    public void setAlliance(int alliance)
    {
        if (alliance == Constants.kALLIANCE_RED)
        {
            team_alliance = alliance;
        }
        else if (alliance == Constants.kALLIANCE_BLUE)
        {
            team_alliance = alliance;
        }
        else
        {
            team_alliance = Constants.kNOT_SET;
        }
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
    public double getRawHeading()
    {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    public void stopMotors()
    {
        intake(0);
        windmillServo.setPower(0);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    //RED ALLIANCE TAGs: 30, 31, 32, 33, 34, 35, 36, 37
    private boolean isRedAllianceTag(int id)
    {
        boolean isRed = (id >= 30 && id <= 37);
        return isRed;
    }
    //BLUE ALLIANCE TAGs: 38, 39, 40, 41, 42, 43, 44, 45
    private boolean isBlueAllianceTag(int id)
    {
        boolean isBlue = (id >= 38 && id <= 45);
        return isBlue;
    }
    public void init(HardwareMap hMap)
    {
        hardwareMap = hMap;
        initIMU();
        initDrive();
        initIntake();
        initShooter();
        initLights();
        initAprilTag();
    }
    private void initIMU()
    {
        // IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
    }
    /*
    * Initialize the hardware variables. Note that the strings used here as parameters
    * to 'get' must correspond to the names assigned during the robot configuration
    * step.
    */
    private void initDrive()
    {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        
    }

    private void initIntake()
    {
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftIntakeServo = hardwareMap.get(CRServo.class, "intake1");
        rightIntakeServo = hardwareMap.get(CRServo.class, "intake2");

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(BRAKE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);

    }
    
    private void initShooter()
    {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");

        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

    }
    private void initLights()
    {

    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }
    }
    public boolean processAprilTag()
    {
        boolean found = false;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections)
        {
            if (detection instanceof AprilTagSingleDetection)
            {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;                
                if(singleDet.metadata != null)
                {
                    if (isRedAllianceTag(singleDet.id) && (team_alliance == Constants.kALLIANCE_RED))
                    {
                        found = true;
                        //telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                        //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    }
                    else if (isBlueAllianceTag(singleDet.id) && (team_alliance == Constants.kALLIANCE_BLUE))
                    {
                        found = true;
                        //telemetry.addLine(String.format("\n==== (ID %d) %s", singleDet.id, singleDet.metadata.name));
                        //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                        //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    }
                    else {
                        //telemetry.addLine(String.format("\n==== (ID %d) Unknown", singleDet.id));
                        //telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", singleDet.center.x, singleDet.center.y));
                    }      
                }
            } 
            else 
            {

                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                if (team_alliance == Constants.kALLIANCE_RED)
                {
                    if ((clusterDet.metadata.name == "RED SCORING") ||
                        (clusterDet.metadata.name == "RED AUDIENCE"))
                    {
                        found = true;
                    }
                }
                else if (team_alliance == Constants.kALLIANCE_BLUE)
                {
                    if ((clusterDet.metadata.name == "BLUE SCORING") ||
                        (clusterDet.metadata.name == "BLUE AUDIENCE"))
                    {
                        found = true;
                    }
                }
            }
        }   // end for() loop
        return found;
    }
}
