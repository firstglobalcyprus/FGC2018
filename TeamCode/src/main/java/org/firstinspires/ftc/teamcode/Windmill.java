package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Windmill  implements  Runnable {

    private Gamepad gamepad;
    private CRServo windmillServo;

    private Thread thread;

    private boolean killed = false;
    private boolean started = false;


    public void init(HardwareMap hm, Gamepad gp) {
        gamepad = gp;
        windmillServo = hm.get(CRServo.class, "windmillServo");

        thread = new Thread(this);
    }

    public void start() {
        thread.start();
        started = true;
    }

    public void kill() {
        killed = true;
        started = false;
    }

    public void run(){
        while (!killed) {
            while (started) {
                if (gamepad.dpad_left) {
                    windmillServo.setPower(1);
                } else {
                    windmillServo.setPower(0);
                }
            }
        }
    }
}

