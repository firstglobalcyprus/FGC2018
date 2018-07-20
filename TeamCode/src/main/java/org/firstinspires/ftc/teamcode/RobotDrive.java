package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

// Controlling the omnidirectional drive of our robot
public class RobotDrive implements Runnable {
    // Have the motor power values all in one place
    private class MotorPowerValues {
        public double p1, p2, p3, p4;

        MotorPowerValues(double _p1, double _p2, double _p3, double _p4) {
            p1 = _p1;
            p2 = _p2;
            p3 = _p3;
            p4 = _p4;
        }
    }

    // The max speed of the robot (0 - 1)
    private double maxSpeed = 0.5;

    // Declare some useful variables
    private Thread thread;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;
    private Gamepad gamepad1;
    private Telemetry telemetry;

    private DcMotor m1, m2, m3, m4;

    // Declare these to keep track the state of the thread
    private boolean killed = false;
    private boolean running = false;


    public void init(LinearOpMode om, HardwareMap hm, Gamepad gp, Telemetry tel) {
        opMode = om;
        hardwareMap = hm;
        gamepad1 = gp;
        telemetry = tel;

        // Motor Setup
        // Setting the direction
        // Initialize the encoders
        // Set ZeroPowerBehavior
        m1 = hardwareMap.get(DcMotor.class, "drive_m1");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m2 = hardwareMap.get(DcMotor.class, "drive_m2");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m3 = hardwareMap.get(DcMotor.class, "drive_m3");
        m3.setDirection(DcMotor.Direction.FORWARD);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m4 = hardwareMap.get(DcMotor.class, "drive_m4");
        m4.setDirection(DcMotor.Direction.FORWARD);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the thread
        thread = new Thread(this);
    }

    // Start the thread
    public void start() {
        thread.start();
        running = true;
    }

    @Override
    // This function runs in parallel with the rest of the code
    public void run() {
        while (!killed) {
            while (running) {
                // Get the stick values
                double gy = -gamepad1.left_stick_y;
                double gx = -gamepad1.left_stick_x;
                double spin = -gamepad1.right_stick_x;

                // Convert the stick values from cartesian to polar coordinates
                double desiredAngle = Math.atan2(gx, gy);
                double mag = Math.sqrt(gy*gy + gx*gx) * maxSpeed;

                // Get the power for each motor
                MotorPowerValues power = getMotorPowerValues(mag, desiredAngle, spin);

                // Set the power for each motor
                m1.setPower(power.p1);
                m2.setPower(power.p2);
                m3.setPower(power.p3);
                m4.setPower(power.p4);
            }
        }
    }

    private MotorPowerValues getMotorPowerValues(double mag, double angle, double spin) {
        // Because the wheels are in a 45 degree angle on the robot I have to subtract that in order to drive correctly
        double driveAngle = angle - Math.PI / 4;

        // Calculate the power for motor 1 and 3
        double p13 = mag * Math.sin(driveAngle);
        // Calculate the power for motor 2 and 4
        double p24 = mag * Math.cos(driveAngle);

        double spinSpeed = spin * maxSpeed;

        // Return the power values for each motor
        return new MotorPowerValues(p13 + spinSpeed, p24 + spinSpeed, p13 - spinSpeed, p24 - spinSpeed);
    }

    // Stop the thread
    public void kill() {
        killed = true;
        running = false;

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    // Get the current max speed
    public double getCurrentSpeed() {
        return maxSpeed;
    }

    // Increase robot's speed
    public void increaseSpeed() {
        if (maxSpeed < 1) maxSpeed += 0.01;
    }

    // Decrease robot's speed
    public void decreaseSpeed() {
        if (maxSpeed > 0.05) maxSpeed -= 0.01;
    }
}
