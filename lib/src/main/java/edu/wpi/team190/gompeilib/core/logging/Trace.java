package edu.wpi.team190.gompeilib.core.logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a method to be profiled for execution time. The time spent inside any method with this
 * annotation will be automatically logged to AdvantageKit.
 */
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface Trace {}
