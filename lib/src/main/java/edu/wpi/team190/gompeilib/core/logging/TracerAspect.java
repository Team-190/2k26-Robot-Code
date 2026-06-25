package edu.wpi.team190.gompeilib.core.logging;

import edu.wpi.first.wpilibj.Timer;
import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.reflect.MethodSignature;
import org.littletonrobotics.junction.Logger;

/**
 * An AspectJ aspect that intercepts calls to methods annotated with @Tracer and logs their
 * execution time to AdvantageKit's logger. This is the core of the automatic profiling system.
 */
@Aspect
public class TracerAspect {

  /**
   * This is the "advice" that runs "around" any method annotated with @Tracer.
   *
   * @param joinPoint The point in the code where the advice is being executed.
   * @return The return value of the original method.
   * @throws Throwable If the original method throws an exception.
   */
  @Around("execution(@Trace * *.*(..))")
  public Object profile(ProceedingJoinPoint joinPoint) throws Throwable {
    // Use FPGA timestamp for high-resolution, synchronized timing
    double startTime = Timer.getFPGATimestamp();

    // Proceed with the original method call
    Object result = joinPoint.proceed();

    double endTime = Timer.getFPGATimestamp();
    double executionTimeMs = (endTime - startTime) * 1000.0;

    // Get method details for logging
    MethodSignature signature = (MethodSignature) joinPoint.getSignature();
    String className = signature.getDeclaringType().getSimpleName();
    String methodName = signature.getName();

    // Log the result to AdvantageKit Logger under the "Tracer/" directory
    String logKey = "Tracer/" + className + "/" + methodName + "MS";
    Logger.recordOutput(logKey, executionTimeMs);

    return result;
  }
}
