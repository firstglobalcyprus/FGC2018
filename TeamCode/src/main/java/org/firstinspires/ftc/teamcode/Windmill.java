package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Windmill  implements  Runnable {

    // Declare some useful variables
    private Gamepad gamepad;
    private CRServo windmillServo;

    private Thread thread;

    // Declare these to keep track the state of the thread
    private boolean killed = false;
    private boolean started = false;


    public void init(HardwareMap hm, Gamepad gp) {
        gamepad = gp;

        // Initialize the servo as continues rotation servo
        windmillServo = hm.get(CRServo.class, "windmillServo");

        // Initialize the thread
        thread = new Thread(this);
    }

    // Start the thread
    public void start() {
        thread.start();
        started = true;
    }

    // Stop the thread
    public void kill() {
        killed = true;
        started = false;
    }

    // This function runs in parallel with the rest of the code
    public void run(){
        while (!killed) {
            while (started) {
                // If dpad_left btn is pressed the servo spins
                if (gamepad.dpad_left) {
                    windmillServo.setPower(1);
                } else {
                    windmillServo.setPower(0);
                }
            }
        }
    }
}

