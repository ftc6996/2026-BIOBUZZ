package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public interface Constants {

    //Alliance
    static final int kNOT_SET = -1;
    static final int kALLIANCE_RED = 2;
    static final int kALLIANCE_BLUE = 1;

    static final int kLAUNCHER_TARGET_VELOCITY = 1230; //1250, 1230
    static final int kLAUNCHER_MIN_VELOCITY = 1215;//1200, 1215

    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    static final double MIN_TURN_SPEED = 0.12;

    static final double kAUTO_TURN_SPEED = .6;

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
}
 