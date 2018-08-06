package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Controlling the claw that grabs the solar panel
public class SolarPanelHandler implements Runnable {
    enum SolarServo {
        lift
    }


    // Preset positions for the servo lift (0 - 1)
    // The higher the value the higher the claw goes
    private static double liftElevatePosition = 0.9;
    private static double liftGripPosition = 0.66;
    private static double liftStorePosition   = 1;

    // Preset positions for the gripper
    private static double gripperClosed    = 0;
    private static double gripperOpen      = 0.7;

    // Store the current position of the servos
    private double liftPosition, gripperPosition;

    // Declare some useful variables
    private Thread       thread;
    private Gamepad      gamepad1;
    private LinearOpMode opMode;
    private ElapsedTime  delayTime = new ElapsedTime();

    private CRServo     gripper;
    private Servo       lift;
    private ElapsedTime lastYButtonPress = new ElapsedTime();

    // Declare these to keep track the state of the thread
    private boolean killed = false;
    private boolean started = false;


    public void init(HardwareMap hm, Gamepad gp, LinearOpMode mode) {
        gamepad1 = gp;
        opMode = mode;

        // Initialize the 2 servos

        gripper = hm.get(CRServo.class, "gripper");
        lift = hm.get(Servo.class, "liftServo");

        initServoPositions();

        // Initialize the tread
        thread = new Thread(this);
        started = false;
    }

    private void initServoPositions() {
        setServoPosition(SolarServo.lift, liftGripPosition, 700);

        setServoPosition(SolarServo.lift, liftStorePosition, 700);

    }

    @Override
    // This function runs in parallel with the rest of the code
    public void run() {

        //the state of the gripper


        while (!killed) {
            while (started) {
                // If the y key is pressed the claw goes up
                // If pressed again the claw goes between the middle and highest position
                // If at any time the a key is pressed the claw opens if closed the goes down
                // If the x key is pressed the claw opens
                // If the b key is pressed the claw closes
                if(gamepad1.y && lastYButtonPress.milliseconds() > 500) {
                    if (liftPosition == liftGripPosition) setServoPosition(SolarServo.lift, liftElevatePosition, 0);
                    else {
                        setServoPosition(SolarServo.lift, liftGripPosition, 400);

                    }

                    lastYButtonPress.reset();
                } else if (gamepad1.a) {

                    setServoPosition(SolarServo.lift, liftStorePosition, 0);
                }

                if(gamepad1.x) {
                    gripper.setPower(1);
                }
                else if(gamepad1.b){
                    gripper.setPower(-1);
                }

                else{
                    gripper.setPower(0);
                }


            }
        }
    }

    public void setServoPosition(SolarServo s, double pos, double t) {
        // Set the position of the servo after reversing the value (1 - pos)
        if (s == SolarServo.lift) {
            lift.setPosition(1 - pos);
            liftPosition = pos;
            delay(t);
        }
    }

    // Start the thread
    public void start() {
        thread.start();
        started = true;


        setServoPosition(SolarServo.lift, liftGripPosition, 0);

    }

    // Stop the thread
    public void kill () {
        killed = true;
    }

    // A function for adding a delay in the code for a set number of ms
    private void delay(double milliseconds) {
        delayTime.reset();
        while (delayTime.milliseconds() < milliseconds && !opMode.isStopRequested());
    }
}
