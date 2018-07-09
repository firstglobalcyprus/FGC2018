

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

    private RobotDrive robot = new RobotDrive();
    private Cubes cubeHandler = new Cubes();
    private SolarPanelHandler panelHandler = new SolarPanelHandler();
    private Windmill windmill = new Windmill();

    private ElapsedTime lastSpeedChange = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);

        Telemetry.Item status = telemetry.addData("Status", "Wait For Start");
        Telemetry.Item pot = telemetry.addData("Pot", 0);
        Telemetry.Item driveSpeed  = telemetry.addData("Drive Speed", "");

        robot.init(this, hardwareMap, gamepad1, telemetry);
        cubeHandler.init(hardwareMap, gamepad1);
        panelHandler.init(hardwareMap, gamepad1, this);
        windmill.init(hardwareMap, gamepad1);

        waitForStart();

        lastSpeedChange.reset();
        status.setValue("Running");
        telemetry.update();

        robot.start();
        cubeHandler.start();
        panelHandler.start();
        windmill.start();

        while (opModeIsActive()) {
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

            pot.setValue(cubeHandler.getLiftPosition());

            driveSpeed.setValue(robot.getCurrentSpeed());
            telemetry.update();
        }

        robot.kill();
        cubeHandler.kill();
        panelHandler.kill();
        windmill.kill();
    }
}
