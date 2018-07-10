package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

public class Cubes implements Runnable {
    private enum CollectorState {
        IN, OUT, STOPPED
    }
    private CollectorState collectorState = CollectorState.STOPPED;
    private ElapsedTime collectorBtnTime = new ElapsedTime();

    private static double liftSpeed = 0.8;
    private static double collectorSpeed = 1;

    private static double minLiftPositionInit = 0.17;
    private static double minLiftPosition = 0.3;
    private static double maxLiftPosition = 0.9;

    private boolean killed = false;
    private boolean started = false;

    private AnalogInput encoderPot;

    private DcMotor collector;
    private DcMotor leftLift, rightLift;

    private Gamepad gamepad1;

    private Thread thread;

    public void init(HardwareMap hardwareMap, Gamepad gp) {
        gamepad1 = gp;

        encoderPot = hardwareMap.get(AnalogInput.class, "encoderPot");

        collector = hardwareMap.get(DcMotor.class, "collector");
        collector.setDirection(DcMotorSimple.Direction.FORWARD);

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

        thread = new Thread(this);
    }

    public void start() {
        thread.start();
        started = true;
        collectorBtnTime.reset();
    }

    public void kill() {
        killed = true;
        started = false;
    }

    public void run () {
        while (!killed) {
            while (started) {
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

    public double getLiftPosition() {
        return 1 - encoderPot.getVoltage() / encoderPot.getMaxVoltage();
    }
}
