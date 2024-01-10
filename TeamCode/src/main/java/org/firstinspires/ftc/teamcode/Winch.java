package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class Winch extends BlocksOpModeCompanion {
    
    // Define features of the winch (a continuous servo)
    static final private int STATE_MOVINGIN = 1;
    static final private int STATE_MOVINGOUT = -1;
    static final private int STATE_STOPPED = 0;
    static int stateOfWinch = STATE_STOPPED;
    static final double SPEED_TOMOVE = 10;
    static final double SPEED_STOP = 0.5;
    static private String servoName = null;
    static private Servo servo    = null;
    static private double currentSpeed  = SPEED_STOP;
    
    // Initialize the winch
    public static void init(String _servoName) {

        servoName = _servoName;

        // Connect to servo
        servo = hardwareMap.get(Servo.class, servoName);

        // Set the servo position the stop speed
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(SPEED_STOP);

        currentSpeed  = SPEED_STOP;
        stateOfWinch = STATE_STOPPED;
    }

  // Ejects a pixel
    public static void hangRobot() {

        moveInward();
        Arm.loosenArmJoints();
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        stop();
    }

    // Stops running servo
    public static void stop() {
        servo.setPosition(SPEED_STOP);
        stateOfWinch = STATE_STOPPED;
        currentSpeed = SPEED_STOP;
    }

    // Starts (or continues) running servo inward
    public static void moveInward() {

        currentSpeed = servo.getPosition();
        if (stateOfWinch != STATE_MOVINGIN) {
            servo.setPosition(STATE_STOPPED);            
            servo.setDirection(Servo.Direction.FORWARD);
            stateOfWinch = STATE_MOVINGIN;
        }
        servo.setPosition(SPEED_TOMOVE);
    }

    // Starts (or continues) running servo outward
    public static void moveOutward() {

        currentSpeed = servo.getPosition();
        if (stateOfWinch != STATE_MOVINGOUT) {
            servo.setPosition(STATE_STOPPED);
            servo.setDirection(Servo.Direction.REVERSE);
            stateOfWinch = STATE_MOVINGOUT;
        }
        servo.setPosition(SPEED_TOMOVE);
    }

    // Returns the name of the joint
    public static String getName() {
        return servoName;
    }
    
    // Loosens the servo
    public static void loosen() {
        // Command below disabled all servos, but all but one were set back to a position after
        // one servo was set to a position:  servo.getController().pwmDisable();

        // Let's loosen this specific servo.
        if (servo instanceof PwmControl)
            ((PwmControl) servo).setPwmDisable();
    }

    
}