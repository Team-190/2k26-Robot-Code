package edu.wpi.team190.gompeilib.core.state.localization;

import edu.wpi.first.apriltag.AprilTag;
import java.util.Set;

/**
 * Defines a logical region of the field in terms of the AprilTags that are considered visible,
 * reliable, or relevant within that area.
 *
 * <p>A {@code FieldZone} contains no estimation or stateful logic on its own. Instead, it serves as
 * a configuration object that describes which vision targets should be associated with a given
 * localization strategy or {@link EstimationRegion}.
 *
 * <p>By separating field structure from estimation behavior, field zones make it possible to:
 *
 * <ul>
 *   <li>Partition the field into overlapping or disjoint regions
 *   <li>Restrict vision updates to specific tag subsets
 *   <li>Swap or combine localization strategies based on robot position
 * </ul>
 *
 * <p>This abstraction is intentionally lightweight and immutable, making it safe to share across
 * subsystems and reuse throughout the localization stack.
 *
 * @param aprilTags the set of AprilTags associated with this field zone
 */
public record FieldZone(Set<AprilTag> aprilTags) {}
