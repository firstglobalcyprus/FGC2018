package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

public class Cubes implements Runnable {
    // This is to keep track of the collector state
    private enum CollectorState {
        IN, OUT, STOPPED
    }
    private CollectorState collectorState = CollectorState.STOPPED;
    private ElapsedTime collectorBtnTime = new ElapsedTime();

    // Max power values
    private static double liftSpeed = 1;
    private static double collectorSpeed = 1;

    // Set the limits for the lift
    private static double minLiftPositionInit = 0.17;
    private static double minLiftPosition = 0.3;
    private static double maxLiftPosition = 0.9;

    // Declare these to keep track the state of the thread
    private boolean killed = false;
    private boolean started = false;

    // Declaring the potentiometer and the motors
    private AnalogInput encoderPot;
    private DcMotor collector;
    private DcMotor leftLift, rightLift;

    private Gamepad gamepad1;
    private Thread thread;

    public void init(HardwareMap hardwareMap, Gamepad gp) {
        gamepad1 = gp;

        // Initializing the potentiometer
        encoderPot = hardwareMap.get(AnalogInput.class, "encoderPot");

        // Initializing the collector motor
        collector = hardwareMap.get(DcMotor.class, "collector");
        collector.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initializing the 2 lift motors
        // Setting their direction
        // Setting ZeroPowerBehavior
        // Initializing the encoders
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initializing the thread
        thread = new Thread(this);
    }

    // Start the thread
    public void start() {
        thread.start();
        started = true;
        collectorBtnTime.reset();
    }

    // Stop the thread
    public void kill() {
        killed = true;
        started = false;
    }

    // This function runs in parallel with the rest of the code
    public void run () {
        while (!killed) {
            while (started) {
                // With R1 and R2 the lift goes up and down respectively
                // The L1 btn toggles the collector (If pressed once it starts spinning to collect, if pressed again it stops)
                // The L2 btn toggles the collector (If pressed once it starts spinning to spit out the cubes, if pressed again it stops)
                if (gamepad1.right_bumper && getLiftPosition() < maxLiftPosition) {
                    leftLift.setPower(liftSpeed);
                    rightLift.setPower(liftSpeed);
                } else if(gamepad1.right_trigger > 0.5 && getLiftPosition() > (gamepad1.left_stick_button ? minLiftPositionInit : minLiftPosition)) {
                    leftLift.setPower(-liftSpeed);
                    rightLift.setPower(-liftSpeed);
                } else {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                }

                if (gamepad1.left_bumper && collectorBtnTime.milliseconds() > 300) {
                    if (collectorState != CollectorState.IN) {
                        collector.setPower(collectorSpeed);
                        collectorState = CollectorState.IN;
                    } else {
                        collector.setPower(0);
                        collectorState = CollectorState.STOPPED;
                    }
                    collectorBtnTime.reset();
                } else if (gamepad1.left_trigger > 0.5 && collectorBtnTime.milliseconds() > 300) {
                    if (collectorState != CollectorState.OUT) {
                        collector.setPower(-collectorSpeed);
                        collectorState = CollectorState.OUT;
                    } else {
                        collector.setPower(0);
                        collectorState = CollectorState.STOPPED;
                    }
                    collectorBtnTime.reset();
                }
            }
        }
    }

    // Get the potentiometers value
    public double getLiftPosition() {
        // Convert the analog voltage of the potentiometer to a value between 0 and 1
        return 1 - encoderPot.getVoltage() / encoderPot.getMaxVoltage();
    }
}
