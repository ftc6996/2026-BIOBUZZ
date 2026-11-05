package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ColorSensor {

    private DigitalChannel dcLedRed = null;
    private DigitalChannel dcLedGreen = null;

    public void init(HardwareMap map) {
        dcLedRed = map.get(DigitalChannel.class, "red");
        dcLedGreen = map.get(DigitalChannel.class, "green");    
        
        //change LED mode from input to output
        dcLedRed.setMode(DigitalChannel.Mode.OUTPUT);
        dcLedGreen.setMode(DigitalChannel.Mode.OUTPUT);
        turnOff();
    }
    public void setGreenOn()
    {
        dcLedGreen.setState(true);
        dcLedRed.setState(false); 
        //LEDstatus = GREENLED;
    }
    public void setRedOn()
    {
        dcLedGreen.setState(false);
        dcLedRed.setState(true);
        //LEDstatus = REDLED;
    }
    public void setYellowOn()
    {
        dcLedGreen.setState(false);
        dcLedRed.setState(false);
        //LEDstatus = YELLOWLED;
    }
    public void turnOff()
    {
        dcLedGreen.setState(true);
        dcLedRed.setState(true);
        //LEDstatus = LEDOFF;
    }    
}