package org.team1126.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableDouble;
import org.team1126.lib.util.command.GRRSubsystem;

import static org.team1126.robot.Constants.STORAGE_MOTOR;


@Logged
public final class Storage extends GRRSubsystem {
//fueling/ spill
    private final SparkMax rollerMotor;
    private SparkMaxConfig rollerConfig;
    private final RelativeEncoder rollerEncoder;
    private SparkClosedLoopController rollerontroller;
    private static final TunableTable tunables = Tunables.getNested("storage");

    private final TunableDouble voltage ;

    public Storage() {

        this.voltage =  tunables.value("Voltage", .5);

        rollerMotor = new SparkMax(STORAGE_MOTOR, SparkLowLevel.MotorType.kBrushless);
        rollerEncoder = rollerMotor.getEncoder();
        rollerConfig = new SparkMaxConfig();

        rollerontroller = rollerMotor.getClosedLoopController();
        rollerConfig.smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .inverted(false)
                .openLoopRampRate(0.25)
                .closedLoopRampRate(0.25);
    }


public Command moveMotorCommand(boolean reverse) {
    return commandBuilder()
            .onExecute(() -> moveMotor(reverse))
            .onEnd(() -> rollerMotor.set(0));
}
    public void moveMotor(boolean reverse) {
        if (reverse) {
            rollerMotor.set(-this.voltage.get());
        }
        else {
            rollerMotor.set(this.voltage.get());
        }
    }
    public Command feedShooter(){
        return commandBuilder()
                .onExecute(() -> moveMotor(false))
                .onEnd(() -> rollerMotor.set(0));
    }

    public Command spill(){
        return commandBuilder()
                .onExecute(() -> moveMotor(true))
                .onEnd(() -> rollerMotor.set(0));
    }
}
