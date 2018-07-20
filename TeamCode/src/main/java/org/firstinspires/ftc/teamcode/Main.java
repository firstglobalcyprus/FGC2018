

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="unicorn", group="FGC-CYPRUS")
public class Main extends LinearOpMode {

    // Creating the the objects for each part of the robot
    private RobotDrive robot = new RobotDrive();
    private Cubes cubeHandler = new Cubes();
    private SolarPanelHandler panelHandler = new SolarPanelHandler();
    private Windmill windmill = new Windmill();

    // Store the time since the last speed change
    private ElapsedTime lastSpeedChange = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Disable telemetries auto clear function
        telemetry.setAutoClear(false);

        // Initialize the telemetry data tha I want to show
        Telemetry.Item status = telemetry.addData("Status", "Wait For Start");
        Telemetry.Item pot = telemetry.addData("Pot", 0);
        Telemetry.Item driveSpeed  = telemetry.addData("Drive Speed", "");

        // Initializing each part of the robot
        robot.init(this, hardwareMap, gamepad1, telemetry);
        cubeHandler.init(hardwareMap, gamepad1);
        panelHandler.init(hardwareMap, gamepad1, this);
        windmill.init(hardwareMap, gamepad1);

        // Wait until the start button is pressed on the Driver Station
        waitForStart();

        lastSpeedChange.reset();
        status.setValue("Running");
        telemetry.update();

        // Make each robot part start working
        robot.start();
        cubeHandler.start();
        panelHandler.start();
        windmill.start();

        // While the stop btn is not pressed
        while (opModeIsActive()) {
            // Used to change the speed of the robot
            // dpad_up   --> increase
            // dpad_down --> decrease
            if (lastSpeedChange.milliseconds() > 20) {
                if (gamepad1.dpad_up) {
                    robot.increaseSpeed();
                    lastSpeedChange.reset();
                }
                else if (gamepad1.dpad_down) {
                    robot.decreaseSpeed();
                    lastSpeedChange.reset();
                }
            }

            // Update the telemetry with new values
            pot.setValue(cubeHandler.getLiftPosition());

            driveSpeed.setValue(robot.getCurrentSpeed());
            telemetry.update();
        }

        // Stop robot parts from working
        robot.kill();
        cubeHandler.kill();
        panelHandler.kill();
        windmill.kill();
    }
}
