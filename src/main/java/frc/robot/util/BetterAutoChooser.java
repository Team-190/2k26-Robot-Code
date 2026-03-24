package frc.robot.util;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAlert;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Setter;

/**
 * An Choreo specific {@code SendableChooser} that allows for the selection of {@link AutoRoutine}s
 * at runtime via a <a
 * href="https://docs.wpilib.org/en/stable/docs/software/dashboards/index.html#dashboards">Dashboard</a>.
 *
 * <p>This chooser takes a <a href="https://en.wikipedia.org/wiki/Lazy_loading">lazy loading</a>
 * approach to {@link AutoRoutine}s, only generating the {@link AutoRoutine} when it is selected.
 * This approach has the benefit of not loading all autos on startup, but also not loading the auto
 * during auto start causing a delay.
 *
 * <p>Once the {@link BetterAutoChooser} is made you can add {@link AutoRoutine}s to it using {@link
 * #addRoutine} or add {@link Command}s to it using {@link #addCmd}. Similar to {@code
 * SendableChooser} this chooser can be added to the {@link
 * edu.wpi.first.wpilibj.smartdashboard.SmartDashboard} using {@code
 * SmartDashboard.putData(Sendable)}.
 *
 * <p>You can set the Robot's autonomous command to the chooser's chosen auto routine via <code>
 * RobotModeTriggers.autonomous.whileTrue(chooser.autoSchedulingCmd());</code>
 */
public class BetterAutoChooser implements Sendable {
  private final String DO_NOTHING_NAME;
  private static final Alert selectedNonexistentAuto =
      ChoreoAlert.alert("Selected an auto that isn't an option", kError);

  private final HashMap<String, Supplier<Command>> autoRoutines = new HashMap<>();
  private final HashMap<String, Supplier<Pose2d>> startingPoses = new HashMap<>();
  private final HashMap<String, Supplier<Command>> onSelectCommand = new HashMap<>();

  private String selected;
  private String[] options = new String[] {};

  private Optional<Alliance> allianceAtGeneration = Optional.empty();
  private String nameAtGeneration;
  private Command generatedCommand = Commands.none();

  private final Consumer<Pose2d> resetPoseConsumer;
  @Setter private boolean resetPose;

  /** Constructs a new {@link BetterAutoChooser}. */
  public BetterAutoChooser(Consumer<Pose2d> resetPoseConsumer) {
    this("Nothing", resetPoseConsumer, true);
  }

  public BetterAutoChooser(Consumer<Pose2d> resetPoseConsumer, boolean resetPose) {
    this("Nothing", resetPoseConsumer, resetPose);
  }

  /**
   * Constructs a new {@link BetterAutoChooser} with the given name for the do-nothing default
   * option.
   *
   * @param doNothingName The option name for the default choice.
   */
  public BetterAutoChooser(
      String doNothingName, Consumer<Pose2d> resetPoseConsumer, boolean resetPose) {
    DO_NOTHING_NAME = doNothingName;
    nameAtGeneration = DO_NOTHING_NAME;
    generatedCommand = Commands.none();
    addCmd(DO_NOTHING_NAME, Commands::none);
    select(DO_NOTHING_NAME);

    this.resetPoseConsumer = resetPoseConsumer;
    this.resetPose = resetPose;
  }

  public BetterAutoChooser(String doNothingName) {
    this(doNothingName, s -> {}, false);
  }

  public BetterAutoChooser() {
    this("Nothing");
  }

  /**
   * @return the name of the default do-nothing option.
   */
  public String getDefaultName() {
    return DO_NOTHING_NAME;
  }

  /**
   * Select a new option in the chooser.
   *
   * <p>This method is called automatically when published as a sendable.
   *
   * @param selectStr The name of the option to select.
   * @return The name of the selected option.
   */
  public String select(String selectStr) {
    return select(selectStr, false);
  }

  private String select(String selectStr, boolean force) {
    selected = selectStr;
    if (selected.equals(nameAtGeneration)
        && allianceAtGeneration.equals(DriverStation.getAlliance())) {
      // early return if the selected auto matches the active auto
      return nameAtGeneration;
    }
    boolean dsValid = DriverStation.isDisabled() && DriverStation.getAlliance().isPresent();
    if (dsValid || force) {
      if (!autoRoutines.containsKey(selected) && !selected.equals(DO_NOTHING_NAME)) {
        selected = DO_NOTHING_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      allianceAtGeneration = DriverStation.getAlliance();
      nameAtGeneration = selected;
      generatedCommand = autoRoutines.get(nameAtGeneration).get().withName(nameAtGeneration);
    } else {
      allianceAtGeneration = Optional.empty();
      nameAtGeneration = DO_NOTHING_NAME;
      generatedCommand = Commands.none();
      return nameAtGeneration; // early return if auto invalid
    }

    if (startingPoses.containsKey(nameAtGeneration) && resetPose) {
      resetPoseConsumer.accept(startingPoses.get(nameAtGeneration).get());
    }

    if (onSelectCommand.containsKey(nameAtGeneration)) {
      CommandScheduler.getInstance()
          .schedule(onSelectCommand.get(nameAtGeneration).get().ignoringDisable(true));
    }

    return nameAtGeneration;
  }

  /**
   * Add an AutoRoutine to the chooser.
   *
   * <p>This is done to load AutoRoutines when and only when they are selected, in order to save
   * memory and file loading time for unused AutoRoutines.
   *
   * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
   *
   * <p>One way to keep this clean is to make an `Autos` class that all of your subsystems/resources
   * are <a href="https://en.wikipedia.org/wiki/Dependency_injection">dependency injected</a> into.
   * Then create methods inside that class that take an {@link AutoFactory} and return an {@link
   * AutoRoutine}.
   *
   * <h3>Example:</h3>
   *
   * <pre><code>
   * BetterAutoChooser chooser;
   * Autos autos = new Autos(swerve, shooter, intake, feeder);
   * public Robot() {
   *   chooser = new BetterAutoChooser("/Choosers");
   *   SmartDashboard.putData(chooser);
   *   // fourPieceRight is a method that accepts an AutoFactory and returns an AutoRoutine.
   *   chooser.addRoutine("4 Piece right", autos::fourPieceRight);
   *   chooser.addRoutine("4 Piece Left", autos::fourPieceLeft);
   *   chooser.addRoutine("3 Piece Close", autos::threePieceClose);
   * }
   * </code></pre>
   *
   * @param name The name of the auto routine.
   * @param generator The function that generates the auto routine.
   * @return This {@link BetterAutoChooser} instance, to allow for method chaining.
   */
  public BetterAutoChooser addRoutine(String name, Supplier<AutoRoutine> generator) {
    autoRoutines.put(name, () -> generator.get().cmd());
    options = autoRoutines.keySet().toArray(new String[0]);
    return this;
  }

  public BetterAutoChooser addRoutine(
      String name, Supplier<AutoRoutine> generator, Supplier<Pose2d> startingPose) {
    startingPoses.put(name, startingPose);
    return addRoutine(name, generator);
  }

  public BetterAutoChooser addRoutineConfig(String name, AutoRoutineConfiguration config) {
    onSelectCommand.put(name, config.prerunCmd());
    return addRoutine(name, config.autoRoutine(), config.startingPose());
  }

  /**
   * Adds a Command to the auto chooser.
   *
   * <p>This is done to load autonomous commands when and only when they are selected, in order to
   * save memory and file loading time for unused autonomous commands.
   *
   * <p>The generators are only run when the DriverStation is disabled and the alliance is known.
   *
   * <h3>Example:</h3>
   *
   * <pre><code>
   * BetterAutoChooser chooser;
   * Autos autos = new Autos(swerve, shooter, intake, feeder);
   * public Robot() {
   *   chooser = new BetterAutoChooser("/Choosers");
   *   SmartDashboard.putData(chooser);
   *   // fourPieceLeft is a method that accepts an AutoFactory and returns a command.
   *   chooser.addCmd("4 Piece left", autos::fourPieceLeft);
   *   chooser.addCmd("Just Shoot", shooter::shoot);
   * }
   * </code></pre>
   *
   * @param name The name of the autonomous command.
   * @param generator The function that generates an autonomous command.
   * @return This {@link BetterAutoChooser} instance, to allow for method chaining.
   * @see BetterAutoChooser#addRoutine
   */
  public BetterAutoChooser addCmd(String name, Supplier<Command> generator) {
    autoRoutines.put(name, generator);
    options = autoRoutines.keySet().toArray(new String[0]);
    return this;
  }

  public BetterAutoChooser addCmd(
      String name, Supplier<Command> generator, Supplier<Pose2d> startingPose) {
    startingPoses.put(name, startingPose);
    return addCmd(name, generator);
  }

  /**
   * Returns the currently selected command.
   *
   * @return The currently selected command.
   */
  public Command selectedCommand() {
    if (RobotBase.isSimulation() && nameAtGeneration == DO_NOTHING_NAME) {
      select(selected, true);
    }
    return generatedCommand;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("String Chooser");
    builder.publishConstBoolean(".controllable", true);
    builder.publishConstString("default", DO_NOTHING_NAME);
    builder.addStringArrayProperty("options", () -> options, null);
    builder.addStringProperty("selected", null, this::select);
    builder.addStringProperty("active", () -> select(selected), null);
  }

  public record AutoRoutineConfiguration(
      Supplier<AutoRoutine> autoRoutine,
      Supplier<Pose2d> startingPose,
      Supplier<Command> prerunCmd) {}
}
