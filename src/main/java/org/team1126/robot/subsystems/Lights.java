package org.team1126.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team1126.lib.math.Math2;
import org.team1126.lib.tunable.TunableTable;
import org.team1126.lib.tunable.Tunables;
import org.team1126.lib.tunable.Tunables.TunableInteger;
import org.team1126.lib.util.Alliance;
import org.team1126.lib.util.Mutable;
import org.team1126.lib.util.command.GRRSubsystem;
import org.team1126.robot.Constants.RioIO;

@Logged
public final class Lights {

    private static final int LENGTH = 28;
    private static final int COUNT = 3;

    private static final TunableTable tunables = Tunables.getNested("lights");

    public static enum Color {
        LEVEL(255, 255, 255),
        HAS_CORAL(255, 255, 255),
        GOOSE(255, 255, 255),
        SCORED(0, 255, 0),
        GOOSE_ASSASSINATION(255, 0, 0),
        BLUE(0, 10, 255),
        RED(255, 0, 0),
        NO_DS(255, 0, 255),
        NO_TAGS(255, 0, 0),
        BAD_LOCATION(255, 144, 0),
        BAD_ROTATION(255, 32, 0),
        SHOOTING(255, 215, 0),
        PURPLE(200, 0, 255),
        CYAN(0, 255, 255),
        LIME(0, 255, 0),
        MAGENTA(255, 0, 255),
        ORANGE(255, 165, 0),
        PINK(255, 192, 203),
        TURQUOISE(64, 224, 208),
        GOLD(255, 215, 0),
        CORAL(255, 127, 80),
        SKY_BLUE(135, 206, 235),
        GREEN(0, 128, 0),
        TEAL(0, 128, 128),
        INDIGO(75, 0, 130),
        LIME_GREEN(50, 205, 50),
        WHITE(255, 255, 255),
        BLACK(0, 0, 0),
        GRAY(128, 128, 128),
        SILVER(192, 192, 192),
        DARK_RED(139, 0, 0),
        DARK_GREEN(0, 100, 0),
        DARK_BLUE(0, 0, 139),
        DARK_CYAN(0, 139, 139),
        DARK_MAGENTA(139, 0, 139),
        LIGHT_RED(255, 99, 71),
        LIGHT_GREEN(144, 238, 144),
        LIGHT_BLUE(173, 216, 230),
        LIGHT_CYAN(224, 255, 255),
        LIGHT_GRAY(211, 211, 211),
        LIGHT_YELLOW(255, 255, 224),
        LIGHT_PINK(255, 182, 193),
        LIGHT_ORANGE(255, 200, 124),
        LIME_YELLOW(50, 205, 50),
        SPRING_GREEN(0, 255, 127),
        AQUA(0, 255, 255),
        NAVY(0, 0, 128),
        MAROON(128, 0, 0),
        OLIVE(128, 128, 0),
        PURPLE_DARK(128, 0, 128),
        FOREST_GREEN(34, 139, 34),
        SEA_GREEN(46, 139, 87),
        STEEL_BLUE(70, 130, 180),
        ROYAL_BLUE(65, 105, 225),
        SLATE_BLUE(106, 90, 205),
        MIDNIGHT_BLUE(25, 25, 112),
        CRIMSON(220, 20, 60),
        SCARLET(255, 36, 0),
        SALMON(250, 128, 114),
        ORANGE_RED(255, 69, 0),
        TOMATO(255, 99, 71),
        KHAKI(240, 230, 200),
        WHEAT(245, 222, 179),
        BEIGE(245, 245, 220),
        LAVENDER(230, 230, 250),
        PLUM(221, 160, 221),
        ORCHID(218, 112, 214),
        VIOLET(238, 130, 238),
        THISTLE(216, 191, 216),
        ROSE(255, 0, 127),
        HOT_PINK(255, 105, 180),
        DEEP_PINK(255, 20, 147),
        CHARTREUSE(127, 255, 0),
        DARK_ORANGE(255, 140, 0),
        BURNT_ORANGE(204, 85, 0),
        RUST(183, 65, 14),
        BROWN(165, 42, 42),
        SADDLE_BROWN(139, 69, 19),
        CHOCOLATE(210, 105, 30),
        PERU(205, 133, 63),
        SANDY_BROWN(244, 164, 96),
        DARK_GOLDENROD(184, 134, 11),
        GOLDENROD(218, 165, 32),
        OFF(0, 0, 0);

        private final TunableInteger r;
        private final TunableInteger g;
        private final TunableInteger b;

        private Color(int r, int g, int b) {
            this.r = tunables.value(name() + "/r", r);
            this.g = tunables.value(name() + "/g", g);
            this.b = tunables.value(name() + "/b", b);
        }

        private int r() {
            return r.get();
        }

        private int g() {
            return g.get();
        }

        private int b() {
            return b.get();
        }
    }

    public final Sides sides;
    public final Top top;

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;

    public Lights() {
        lights = new AddressableLED(RioIO.LIGHTS);
        buffer = new AddressableLEDBuffer(LENGTH * COUNT);

        lights.setLength(buffer.getLength());
        lights.start();

        sides = new Sides();
        top = new Top();
    }

    public void update() {
        lights.setData(buffer);
    }

    /**
     * Displays the pre-match animation.
     * @param robotPose The robot's current pose.
     * @param seesAprilTag If the robot has seen an AprilTag since the last loop.
     * @param defaultAuto If the default auto is selected.
     */
    public Command preMatch(Supplier<Pose2d> robotPose, BooleanSupplier seesAprilTag, BooleanSupplier defaultAuto) {
        return parallel(sides.preMatch(defaultAuto), top.preMatch(robotPose, seesAprilTag))
            .until(DriverStation::isEnabled)
            .ignoringDisable(true)
            .withName("Lights.disabled()");
    }

    @Logged
    public final class Sides extends GRRSubsystem {

        private Sides() {}

        /**
         * Modifies the entire side LED strips to be a single color.
         * @param color The color to apply.
         */
        private void setBoth(Color color) {
            for (int i = 0; i < LENGTH; i++) {
                setBoth(i, color);
            }
        }

        /**
         * Modifies the buffer with values mirrored across the "center" of the LED strip.
         * @param i The index of the buffer to modify.
         * @param color The color to apply.
         */
        private void setBoth(int i, Color color) {
            setBoth(i, color.r(), color.g(), color.b());
        }

        /**
         * Modifies the buffer with values mirrored across the "center" of the LED strip.
         * @param i The index of the buffer to modify.
         * @param r Red value from {@code 0} to {@code 255}.
         * @param g Green value from {@code 0} to {@code 255}.
         * @param b Blue value from {@code 0} to {@code 255}.
         */
        private void setBoth(int i, int r, int g, int b) {
            setSingle(false, i, r, g, b);
            setSingle(true, i, r, g, b);
        }

        /**
         * Sets a single side's LED to a specified color.
         * @param left {@code true} for the left side, {@code false} for the right side.
         * @param i The index of the strip.
         * @param color The color to apply.
         */
        private void setSingle(boolean left, int i, Color color) {
            setSingle(left, i, color.r(), color.g(), color.b());
        }

        /**
         * Sets a single side's LED to a specified color.
         * @param left {@code true} for the left side, {@code false} for the right side.
         * @param i The index of the strip.
         * @param r Red value from {@code 0} to {@code 255}.
         * @param g Green value from {@code 0} to {@code 255}.
         * @param b Blue value from {@code 0} to {@code 255}.
         */
        private void setSingle(boolean left, int i, int r, int g, int b) {
            if (i >= LENGTH) return;
            buffer.setRGB(!left ? i : buffer.getLength() - i - 1, r, g, b);
        }

        public Command setSolidRed() {
            return commandBuilder()
                .onInitialize(() -> setBoth(Color.RED))
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.setSolidRed()");
        }

        public Command shooting() {
            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        for (int j = 0; j <= 1; j++) {
                            boolean blink = true; // selection.isScoring() && (selection.isLeft() ? 0 : 1) == j;

                            if ((blink ? RobotController.getRSLState() : true) && i < ((28 / 4) * 1)) {
                                setSingle(j == 0, i, Color.LEVEL);
                            } else {
                                setSingle(j == 0, i, Color.OFF);
                            }
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.levelSelection()");
        }

        /**
         * Displays the percentage of the climber deploy position.
         * @param percent The climber's distance from the deploy position as a percent from [0.0, 1.0].
         */
        public Command climbPercent(DoubleSupplier percent) {
            return commandBuilder()
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        setBoth(
                            i,
                            i > Math.ceil(percent.getAsDouble() * LENGTH)
                                ? Color.OFF
                                : Alliance.isBlue()
                                    ? Color.BLUE
                                    : Color.RED
                        );
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.climbPercent()");
        }

        /**
         * Displays the flames animation.
         */
        public Command flames(boolean allianceColors) {
            Mutable<Integer> count = new Mutable<>(0);
            int[] state = new int[LENGTH];

            return commandBuilder()
                .onInitialize(() -> {
                    count.value = 0;
                    for (int i = 0; i < state.length; i++) {
                        state[i] = 0;
                    }
                })
                .onExecute(() -> {
                    for (int i = 0; i < LENGTH; i++) {
                        state[i] = Math.max(0, state[i] - 16);
                    }

                    for (int i = LENGTH - 1; i >= 2; i--) {
                        state[i] = (state[i - 1] + state[i - 2] + state[i - 2]) / 3;
                    }

                    if (count.value++ % 2 == 0) {
                        int i = (int) Math2.random(3.0);
                        state[i] = (int) Math.min(255.0, state[i] + Math2.random(200.0, 255.0));
                    }

                    for (int i = 0; i < LENGTH; i++) {
                        int heat = Math.min(255, state[i] + (Math.random() < 0.05 && state[i] > 16 ? 32 : 0));
                        int ramp = (((int) Math.round((heat / 255.0) * 191.0)) & 63) << 2;
                        if (heat > 170) {
                            if (allianceColors) {
                                if (Alliance.isBlue()) setBoth(i, ramp, ramp, 255);
                                else setBoth(i, 255, ramp, ramp);
                            } else {
                                setBoth(i, 255, 255, ramp / 2);
                            }
                        } else if (heat > 84) {
                            if (allianceColors) {
                                if (Alliance.isBlue()) setBoth(i, ramp / 4, 0, 255);
                                else setBoth(i, 255, 0, ramp / 4);
                            } else {
                                setBoth(i, 255, ramp / 2, 0);
                            }
                        } else {
                            if (allianceColors && Alliance.isBlue()) {
                                setBoth(i, 0, 0, ramp);
                            } else {
                                setBoth(i, ramp, 0, 0);
                            }
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.flames()");
        }

        /**
         * Displays the pre-match animation.
         * @param defaultAuto If the default auto is selected.
         */
        public Command preMatch(BooleanSupplier defaultAuto) {
            Timer timer = new Timer();

            return commandBuilder()
                .onExecute(() -> {
                    if (defaultAuto.getAsBoolean()) {
                        timer.stop();
                        timer.reset();
                    } else {
                        timer.start();
                    }

                    if (timer.get() % 0.12 > 0.06 || timer.hasElapsed(1.0)) {
                        setBoth(Alliance.isBlue() ? Color.BLUE : Color.RED);
                    } else {
                        setBoth(Color.OFF);
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.preMatch()");
        }

        /**
         * Turns the lights off.
         */
        public Command off() {
            return commandBuilder()
                .onInitialize(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.off()");
        }

        /**
         * Chase animation with a specified color.
         * @param color The color to chase with.
         */
        public Command chase(Color color) {
            Mutable<Integer> position = new Mutable<>(0);
            Mutable<Integer> frameCounter = new Mutable<>(0);
            final double SPEED = .75; // Increase this number to slow down

            return commandBuilder()
                .onInitialize(() -> {
                    position.value = 0;
                    frameCounter.value = 0;
                })
                .onExecute(() -> {
                    // Clear all LEDs for both sides
                    for (int i = 0; i < LENGTH; i++) {
                        for (int j = 0; j <= 1; j++) {
                            setSingle(j == 0, i, Color.OFF);
                        }
                    }

                    // Set the chasing LED and a few behind it for a trail
                    for (int trail = 0; trail < 3; trail++) {
                        int index = (position.value - trail + LENGTH) % LENGTH;
                        int fadeR = (trail == 0) ? color.r() : (color.r() * (3 - trail)) / 3;
                        int fadeG = (trail == 0) ? color.g() : (color.g() * (3 - trail)) / 3;
                        int fadeB = (trail == 0) ? color.b() : (color.b() * (3 - trail)) / 3;

                        // Apply to both sides
                        for (int j = 0; j <= 1; j++) {
                            setSingle(j == 0, index, fadeR, fadeG, fadeB);
                        }
                    }

                    // Update position only every SPEED frames
                    if (frameCounter.value++ >= SPEED) {
                        frameCounter.value = 0;
                        position.value = (position.value + 1) % LENGTH;
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.chase()");
        }

        /**
         * Back and forth chase animation for the right segment.
         * @param color The color to chase with.
         */
        public Command backAndForth(Color color) {
            Mutable<Integer> position = new Mutable<>(0);
            Mutable<Boolean> movingRight = new Mutable<>(true);

            return commandBuilder()
                .onInitialize(() -> {
                    position.value = 0;
                    movingRight.value = true;
                })
                .onExecute(() -> {
                    // Clear all LEDs
                    for (int i = 0; i < LENGTH; i++) {
                        for (int j = 0; j <= 1; j++) {
                            setSingle(j == 0, i, Color.OFF);
                        }
                    }

                    // Draw the chasing LED and trail on right segment
                    for (int trail = 0; trail < 3; trail++) {
                        int index = position.value - trail;
                        if (index >= 0 && index < LENGTH) {
                            int fadeR = (trail == 0) ? color.r() : (color.r() * (3 - trail)) / 3;
                            int fadeG = (trail == 0) ? color.g() : (color.g() * (3 - trail)) / 3;
                            int fadeB = (trail == 0) ? color.b() : (color.b() * (3 - trail)) / 3;
                            setSingle(false, index, fadeR, fadeG, fadeB); // Right side
                        }
                    }

                    // Move in current direction
                    if (movingRight.value) {
                        position.value++;
                        // Reached right edge, reverse direction
                        if (position.value >= LENGTH) {
                            position.value = LENGTH - 1;
                            movingRight.value = false;
                        }
                    } else {
                        position.value--;
                        // Reached left edge, reverse direction
                        if (position.value < 0) {
                            position.value = 0;
                            movingRight.value = true;
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.backAndForth()");
        }

        /**
         * Gradient chase animation - moves a blob with a gradient fade.
         * @param color The base color of the gradient blob.
         */
        public Command gradientChase(Color color) {
            Mutable<Integer> position = new Mutable<>(0);
            Mutable<Boolean> movingRight = new Mutable<>(true);
            Mutable<Integer> frameCounter = new Mutable<>(0);
            final int SPEED = 2; // Increase this number to slow down (update every N frames)

            return commandBuilder()
                .onInitialize(() -> {
                    position.value = 0;
                    movingRight.value = true;
                    frameCounter.value = 0;
                })
                .onExecute(() -> {
                    // Clear all LEDs
                    for (int i = 0; i < LENGTH; i++) {
                        for (int j = 0; j <= 1; j++) {
                            setSingle(j == 0, i, Color.OFF);
                        }
                    }

                    // Draw gradient blob with center at position
                    // Gradient extends 3 LEDs in each direction from center
                    final int GRADIENT_WIDTH = 5;
                    for (int offset = -GRADIENT_WIDTH; offset <= GRADIENT_WIDTH; offset++) {
                        int index = position.value + offset;
                        if (index >= 0 && index < LENGTH) {
                            // Distance from center (0 = center, GRADIENT_WIDTH = edge)
                            int distance = Math.abs(offset);
                            // Gradient factor: 1.0 at center, 0.0 at edges
                            double gradientFactor = 1.0 - ((double) distance / GRADIENT_WIDTH);
                            // Cube the gradient for very intense/sharp falloff
                            gradientFactor = gradientFactor * gradientFactor * gradientFactor;

                            // Create lighter shades of the same color: multiply RGB by gradient factor
                            int r = (int) (color.r() * gradientFactor);
                            int g = (int) (color.g() * gradientFactor);
                            int b = (int) (color.b() * gradientFactor);

                            setSingle(false, index, r, g, b); // Right side
                        }
                    }

                    // Update position only every SPEED frames
                    if (frameCounter.value++ >= SPEED) {
                        frameCounter.value = 0;

                        // Move in current direction
                        if (movingRight.value) {
                            position.value++;
                            // Reached right edge, reverse direction
                            if (position.value >= LENGTH) {
                                position.value = LENGTH - 1;
                                movingRight.value = false;
                            }
                        } else {
                            position.value--;
                            // Reached left edge, reverse direction
                            if (position.value < 0) {
                                position.value = 0;
                                movingRight.value = true;
                            }
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.gradientChase()");
        }

        /**
         * Color cycling chase: a chase that cycles through an array of colors. As the chase moves,
         * it changes the segment color to match the current chase color, creating a cycling effect.
         * @param colors Array of colors to cycle through in sequence.
         */
        public Command colorCyclingChase(Color... colors) {
            Mutable<Integer> position = new Mutable<>(0);
            Mutable<Integer> colorIndex = new Mutable<>(0);
            Mutable<Integer> frameCounter = new Mutable<>(0);
            final int SPEED = 2; // Update position every 2 frames
            final int TRAIL_LENGTH = 3;

            return commandBuilder()
                .onInitialize(() -> {
                    position.value = 0;
                    colorIndex.value = 0;
                    frameCounter.value = 0;
                })
                .onExecute(() -> {
                    if (colors.length == 0) return; // Safety check

                    Color currentColor = colors[colorIndex.value];
                    Color previousColor = colors[(colorIndex.value - 1 + colors.length) % colors.length];

                    // Fill entire segment with previous color initially
                    for (int i = 0; i < LENGTH; i++) {
                        for (int j = 0; j <= 1; j++) {
                            setSingle(j == 0, i, previousColor);
                        }
                    }

                    // Behind the chase (already covered), fill with current color
                    for (int i = 0; i < position.value; i++) {
                        for (int j = 0; j <= 1; j++) {
                            setSingle(j == 0, i, currentColor);
                        }
                    }

                    // Draw the chase trail itself (bright version of current color)
                    for (int offset = -TRAIL_LENGTH; offset <= TRAIL_LENGTH; offset++) {
                        int index = position.value + offset;
                        if (index >= 0 && index < LENGTH) {
                            // Create brightness boost by reducing fade based on distance from center
                            double brightnessMultiplier = 1.0 - (Math.abs(offset) / (double) (TRAIL_LENGTH + 1)) * 0.3;

                            int chaseR = (int) (currentColor.r() * brightnessMultiplier);
                            int chaseG = (int) (currentColor.g() * brightnessMultiplier);
                            int chaseB = (int) (currentColor.b() * brightnessMultiplier);

                            setSingle(true, index, chaseR, chaseG, chaseB); // Left side
                            setSingle(false, index, chaseR, chaseG, chaseB); // Right side
                        }
                    }

                    // Update position every SPEED frames
                    frameCounter.value++;
                    if (frameCounter.value >= SPEED) {
                        frameCounter.value = 0;
                        position.value++;

                        // When reaching the end, cycle to next color
                        if (position.value >= LENGTH) {
                            position.value = 0;
                            colorIndex.value = (colorIndex.value + 1) % colors.length;
                        }
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.colorCyclingChase()");
        }

        /**
         * Fade animation that cycles through: OFF → color1 fades in → color1 fades out → OFF → color2 fades in → color2 fades out → repeat
         * @param color1 The first color to fade in and out.
         * @param color2 The second color to fade in and out.
         */
        public Command fade(Color color1, Color color2) {
            Mutable<Integer> frameCounter = new Mutable<>(0);
            final int FADE_DURATION = 35; // Frames for fade in or fade out
            final int OFF_DURATION = 5; // Frames to stay off between fades
            final int CYCLE_DURATION = (FADE_DURATION * 2 + OFF_DURATION) * 2; // Full cycle with both colors

            return commandBuilder()
                .onInitialize(() -> frameCounter.value = 0)
                .onExecute(() -> {
                    // Increment frame counter
                    frameCounter.value++;
                    int cyclePos = frameCounter.value % CYCLE_DURATION;

                    int r = 0,
                        g = 0,
                        b = 0;

                    // Color 1 sequence: OFF → fade in → fade out → OFF
                    if (cyclePos < FADE_DURATION) {
                        // Fade color1 in
                        double factor = cyclePos / (double) FADE_DURATION;
                        r = (int) (color1.r() * factor);
                        g = (int) (color1.g() * factor);
                        b = (int) (color1.b() * factor);
                    } else if (cyclePos < FADE_DURATION * 2) {
                        // Fade color1 out
                        double factor = (FADE_DURATION * 2 - cyclePos) / (double) FADE_DURATION;
                        r = (int) (color1.r() * factor);
                        g = (int) (color1.g() * factor);
                        b = (int) (color1.b() * factor);
                    } else if (cyclePos < FADE_DURATION * 2 + OFF_DURATION) {
                        // Off period
                        r = 0;
                        g = 0;
                        b = 0;
                    }
                    // Color 2 sequence: OFF → fade in → fade out → OFF
                    else if (cyclePos < FADE_DURATION * 3 + OFF_DURATION) {
                        // Fade color2 in
                        double factor = (cyclePos - FADE_DURATION * 2 - OFF_DURATION) / (double) FADE_DURATION;
                        r = (int) (color2.r() * factor);
                        g = (int) (color2.g() * factor);
                        b = (int) (color2.b() * factor);
                    } else if (cyclePos < FADE_DURATION * 4 + OFF_DURATION) {
                        // Fade color2 out
                        double factor = (FADE_DURATION * 4 + OFF_DURATION - cyclePos) / (double) FADE_DURATION;
                        r = (int) (color2.r() * factor);
                        g = (int) (color2.g() * factor);
                        b = (int) (color2.b() * factor);
                    }

                    // Apply the faded color to all LEDs on both sides
                    for (int i = 0; i < LENGTH; i++) {
                        setBoth(i, r, g, b);
                    }
                })
                .onEnd(() -> setBoth(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.fade()");
        }
    }

    @Logged
    public final class Top extends GRRSubsystem {

        private Top() {}

        /**
         * Modifies the entire side LED strips to be a single color.
         * @param color The color to apply.
         */
        private void set(Color color) {
            for (int i = 0; i < LENGTH; i++) set(i, color);
        }

        private void set(int i, Color color) {
            if (i >= LENGTH) return;
            buffer.setRGB(LENGTH + i, color.r(), color.g(), color.b());
        }

        private void set(int i, int r, int g, int b) {
            if (i >= LENGTH) return;
            buffer.setRGB(LENGTH + i, r, g, b);
        }

        public Command setSolidRed() {
            return commandBuilder()
                .onInitialize(() -> set(Color.RED))
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.setSolidRed()");
        }

        /**
         * Displays the coral state.
         */
        public Command coralDisplay(BooleanSupplier hasCoral, BooleanSupplier goosing, DoubleSupplier goosePosition) {
            final double GOOSE_RANGE = 0.15;
            final double HALF_RANGE = GOOSE_RANGE / 2.0;

            Timer timer = new Timer();

            return sequence(
                run(() -> set(Color.OFF)).until(hasCoral::getAsBoolean),
                run(() -> {
                    if (!goosing.getAsBoolean()) {
                        set(RobotController.getRSLState() ? Color.HAS_CORAL : Color.OFF);
                    } else {
                        double position = goosePosition.getAsDouble();
                        double percent =
                            (MathUtil.clamp(Math.abs(position), 0.5 - HALF_RANGE, 0.5 + HALF_RANGE)
                                - (0.5 - HALF_RANGE))
                            * (1.0 / GOOSE_RANGE);
                        if (position < 0.0) percent = 1.0 - percent;
                        int closestLED = (int) Math.round(percent * (LENGTH - 1));
                        for (int i = 0; i < LENGTH; i++) {
                            if (Math.abs(closestLED - i) <= 1) {
                                set(i, Color.GOOSE);
                            } else {
                                set(i, Color.OFF);
                            }
                        }
                    }
                }).until(() -> !hasCoral.getAsBoolean()),
                run(() -> set(timer.get() % 0.2 > 0.1 ? Color.SCORED : Color.OFF))
                    .beforeStarting(timer::restart)
                    .withTimeout(1.5)
            )
                .repeatedly()
                .ignoringDisable(true)
                .withName("Lights.Top.coralDisplay()");
        }

        /**
         * Displays the pre-match animation.
         * @param robotPose The robot's current pose.
         * @param seesAprilTag If the robot has seen an AprilTag since the last loop.
         */
        public Command preMatch(Supplier<Pose2d> robotPose, BooleanSupplier seesAprilTag) {
            final double LOCATION_TOL = 0.05;
            final double ROTATION_TOL = Math.toRadians(0.4);

            Debouncer tag = new Debouncer(0.5, DebounceType.kFalling);
            Debouncer location = new Debouncer(0.5, DebounceType.kFalling);
            Debouncer rotation = new Debouncer(0.5, DebounceType.kFalling);

            Debouncer sightDebounce = new Debouncer(0.06, DebounceType.kFalling);
            SlewRateLimiter center = new SlewRateLimiter(ROTATION_TOL * 2.0);
            Mutable<Pose2d> lastPose = new Mutable<>(Pose2d.kZero);
            Mutable<Double> errorTime = new Mutable<>(-1.0);
            List<Color> errors = new ArrayList<>();

            return commandBuilder()
                .onInitialize(() -> {
                    Pose2d pose = robotPose.get();
                    center.reset(pose.getRotation().getRadians());
                    lastPose.value = pose;
                    errorTime.value = -1.0;
                })
                .onExecute(() -> {
                    if (!DriverStation.isDSAttached()) {
                        set(Color.NO_DS);
                        return;
                    }

                    errors.clear();
                    Pose2d pose = robotPose.get();

                    if (tag.calculate(!sightDebounce.calculate(seesAprilTag.getAsBoolean()))) {
                        errors.add(Color.NO_TAGS);
                    } else if (
                        location.calculate(
                            !Math2.isNear(lastPose.value.getTranslation(), pose.getTranslation(), LOCATION_TOL)
                        )
                    ) {
                        errors.add(Color.BAD_LOCATION);
                    } else if (
                        rotation.calculate(
                            !Math2.isNear(lastPose.value.getRotation(), pose.getRotation(), ROTATION_TOL)
                        )
                    ) {
                        errors.add(Color.BAD_ROTATION);
                    }

                    lastPose.value = pose;
                    if (!errors.isEmpty()) {
                        double now = Timer.getFPGATimestamp();
                        if (errorTime.value < 0.0) errorTime.value = now;

                        double period = (now - errorTime.value) % 0.2;
                        if (period < 0.15) {
                            int i = (int) Math.floor((period / 0.15) * errors.size());
                            set(errors.get(Math.min(i, errors.size())));
                        } else {
                            set(Color.OFF);
                        }
                    } else {
                        double radians = pose.getRotation().getRadians();

                        if (errorTime.value > 0.0) {
                            errorTime.value = -1.0;
                            center.reset(radians);
                        }

                        double view = Math.toRadians(1.0);
                        double percent = (center.calculate(radians) + (view / 2.0) - radians) / view;
                        SmartDashboard.putNumber("Lights.Top.preMatch/percent", percent);
                        int closestLED = (int) Math.round(percent * (LENGTH - 1));
                        SmartDashboard.putNumber("Lights.Top.preMatch/closestLED", closestLED);
                        Color alliance = Alliance.isBlue() ? Color.BLUE : Color.RED;
                        for (int i = 0; i < LENGTH; i++) {
                            if (Math.abs(closestLED - i) <= 1) {
                                set(i, alliance);
                            } else {
                                set(i, Color.OFF);
                            }
                        }
                    }
                })
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.preMatch()");
        }

        /**
         * Displays the climbing animation.
         */
        public Command climbing(BooleanSupplier isDeployed) {
            return commandBuilder()
                .onExecute(() ->
                    set(
                        !RobotController.getRSLState() && isDeployed.getAsBoolean()
                            ? Color.OFF
                            : (Alliance.isBlue() ? Color.BLUE : Color.RED)
                    )
                )
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Sides.climbing()");
        }

        /**
         * Turns the lights off.
         */
        public Command off() {
            return commandBuilder()
                .onInitialize(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.off()");
        }

        /**
         * Knight Rider effect: the entire segment is filled with a base color, and a brighter
         * chase animates back and forth on top of it to create the classic Knight Rider look.
         * @param baseColor The base color that fills the entire segment.
         * @param chaseColor The brighter chase color that animates back and forth.
         */
        public Command knightRider(Color baseColor, Color chaseColor) {
            Mutable<Integer> position = new Mutable<>(0);
            Mutable<Boolean> movingRight = new Mutable<>(true);
            Mutable<Integer> frameCounter = new Mutable<>(0);
            final int SPEED = 2; // Increase this number to slow down

            return commandBuilder()
                .onInitialize(() -> {
                    position.value = 0;
                    movingRight.value = true;
                    frameCounter.value = 0;
                })
                .onExecute(() -> {
                    // Fill entire segment with base color
                    for (int i = 0; i < LENGTH; i++) {
                        set(i, baseColor);
                    }

                    // Draw solid chase blob on top of base color
                    final int CHASE_WIDTH = 3;
                    for (int offset = -CHASE_WIDTH; offset <= CHASE_WIDTH; offset++) {
                        int index = position.value + offset;
                        if (index >= 0 && index < LENGTH) {
                            set(index, chaseColor);
                        }
                    }

                    // Update position only every SPEED frames
                    if (frameCounter.value++ >= SPEED) {
                        frameCounter.value = 0;

                        // Move in current direction
                        if (movingRight.value) {
                            position.value++;
                            // Reached right edge, reverse direction
                            if (position.value >= LENGTH) {
                                position.value = LENGTH - 1;
                                movingRight.value = false;
                            }
                        } else {
                            position.value--;
                            // Reached left edge, reverse direction
                            if (position.value < 0) {
                                position.value = 0;
                                movingRight.value = true;
                            }
                        }
                    }
                })
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.knightRider()");
        }

        /**
         * Converging chase animation where LEDs chase from both ends of the top segment toward the middle.
         * @param color The color to chase with.
         */
        public Command convergeToMiddle(Color color) {
            Mutable<Integer> position = new Mutable<>(0);
            Mutable<Integer> frameCounter = new Mutable<>(0);
            final int MIDDLE = LENGTH / 2;
            final double SPEED = 2.0; // Increase this number to slow down

            return commandBuilder()
                .onInitialize(() -> {
                    position.value = 0;
                    frameCounter.value = 0;
                })
                .onExecute(() -> {
                    // Clear all LEDs
                    for (int i = 0; i < LENGTH; i++) {
                        set(i, Color.OFF);
                    }

                    // Chase from left edge (0) toward middle
                    for (int trail = 0; trail < 3; trail++) {
                        int index = position.value - trail;
                        if (index >= 0 && index < MIDDLE) {
                            int fadeR = (trail == 0) ? color.r() : (color.r() * (3 - trail)) / 3;
                            int fadeG = (trail == 0) ? color.g() : (color.g() * (3 - trail)) / 3;
                            int fadeB = (trail == 0) ? color.b() : (color.b() * (3 - trail)) / 3;
                            set(index, fadeR, fadeG, fadeB);
                        }
                    }

                    // Chase from right edge (LENGTH-1) toward middle, mirrored
                    for (int trail = 0; trail < 3; trail++) {
                        int index = LENGTH - 1 - (position.value - trail);
                        if (index >= MIDDLE && index < LENGTH) {
                            int fadeR = (trail == 0) ? color.r() : (color.r() * (3 - trail)) / 3;
                            int fadeG = (trail == 0) ? color.g() : (color.g() * (3 - trail)) / 3;
                            int fadeB = (trail == 0) ? color.b() : (color.b() * (3 - trail)) / 3;
                            set(index, fadeR, fadeG, fadeB);
                        }
                    }

                    // Update position only every SPEED frames
                    if (frameCounter.value++ >= SPEED) {
                        frameCounter.value = 0;

                        // Loop animation
                        if (position.value < MIDDLE) {
                            position.value++;
                        } else {
                            position.value = 0;
                        }
                    }
                })
                .onEnd(() -> set(Color.OFF))
                .ignoringDisable(true)
                .withName("Lights.Top.convergeToMiddle()");
        }
    }
}
