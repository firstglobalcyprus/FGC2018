package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controlling the claw that grabs the solar panel
public class SolarPanelHandler implements Runnable {
    // Preset positions for the servo lift (0 - 1)
    // The higher the value the lower the claw goes
    private static double liftDownPosition = 1;
    private static double liftGripPosition = 0.5;
    private static double liftUpPosition   = 0.3;

    // Preset positions for the gripper
    private static double gripperClosed    = 1;
    private static double gripperOpen      = 0;

    // Store the current position of the servos
    private double liftPosition, gripperPosition;

    // Declare some useful variables
    private Thread       thread;
    private Gamepad      gamepad1;
    private LinearOpMode opMode;
    private ElapsedTime  delayTime = new ElapsedTime();

    private Servo       gripper;
    private Servo       lift;
    private ElapsedTime lastYButtonPress = new ElapsedTime();

    // Declare these to keep track the state of the thread
    private boolean killed = false;
    private boolean started = false;


    public void init(HardwareMap hm, Gamepad gp, LinearOpMode mode) {
        gamepad1 = gp;
        opMode = mode;

        // Initialize the 2 servos
        gripper = hm.get(Servo.class, "gripper");
        lift = hm.get(Servo.class, "liftServo");

        lift.setPosition(liftDownPosition);
        liftPosition = liftDownPosition;

        gripperPosition = gripperOpen;

        // Initialize the tread
        thread = new Thread(this);
    }

    @Override
    // This function runs in parallel with the rest of the code
    public void run() {
        while (!killed) {
            while (started) {
                // If the y key is pressed the claw goes up
                // If pressed again the claw goes between the middle and highest position
                // If at any time the a key is pressed the claw opens if closed the goes down
                // If the x key is pressed the claw opens
                // If the b key is pressed the claw closes
                if(gamepad1.y && lastYButtonPress.milliseconds() > 300) {
                    if (liftPosition == liftGripPosition) {
                        lift.setPosition(liftUpPosition);
                        liftPosition = liftUpPosition;
                    }
                    else {
                        lift.setPosition(liftGripPosition);
                        liftPosition = liftGripPosition;
                    }
                    lastYButtonPress.reset();
                } else if (gamepad1.a) {
                    gripper.setPosition(gripperOpen);
                    gripperPosition = gripperOpen;
                    delay(700);

                    lift.setPosition(liftDownPosition);
                    liftPosition = liftDownPosition;
                }

                if(gamepad1.x) {
                    gripper.setPosition(gripperOpen);
                    gripperPosition = gripperOpen;
                } else if (gamepad1.b) {
                    if (liftPosition != liftDownPosition) {
                        gripper.setPosition(gripperClosed);
                        gripperPosition = gripperClosed;
                    }
                }
            }
        }
    }

    // Start the thread
    public void start() {
        thread.start();
        started = true;
    }

    // Stop the thread
    public void kill () {
        killed = true;
    }

    // A function for adding a delay in the code for a set number of ms
    private void delay(double milliseconds) {
        delayTime.reset();
        while (delayTime.milliseconds() < milliseconds && opMode.opModeIsActive());
    }
}
