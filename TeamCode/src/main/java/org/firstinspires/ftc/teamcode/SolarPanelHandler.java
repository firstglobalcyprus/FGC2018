package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SolarPanelHandler implements Runnable {
    private static double liftDownPosition = 1;
    private static double liftGripPosition = 0.5;
    private static double liftUpPosition   = 0.3;

    private static double gripperClosed    = 1;
    private static double gripperOpen      = 0;

    private double liftPosition, gripperPosition;

    private Thread       thread;
    private Gamepad      gamepad1;
    private LinearOpMode opMode;
    private ElapsedTime  delayTime = new ElapsedTime();

    private boolean killed = false;
    private boolean started = false;

    private Servo       gripper;
    private Servo       lift;
    private ElapsedTime lastYButtonPress = new ElapsedTime();

    public void init(HardwareMap hm, Gamepad gp, LinearOpMode mode) {
        thread = new Thread(this);
        gamepad1 = gp;
        opMode = mode;

        gripper = hm.get(Servo.class, "gripper");
        lift = hm.get(Servo.class, "liftServo");

        lift.setPosition(liftDownPosition);
        liftPosition = liftDownPosition;

        gripperPosition = gripperOpen;
    }

    public void run() {
        while (!killed) {
            while (started) {
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

    public void start() {
        thread.start();
        started = true;
    }

    public void kill () {
        killed = true;
    }

    private void delay(double milliseconds) {
        delayTime.reset();
        while (delayTime.milliseconds() < milliseconds && opMode.opModeIsActive());
    }
}
