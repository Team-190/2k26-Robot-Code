package edu.wpi.team190.gompeilib.core.logging;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;

class TraceTest {

  @Trace
  void annotatedMethod() {}

  void nonAnnotatedMethod() {}

  @Test
  void testRetentionPolicyIsRuntime() {
    Retention retention = Trace.class.getAnnotation(Retention.class);
    assertNotNull(retention);
    assertEquals(RetentionPolicy.RUNTIME, retention.value());
  }

  @Test
  void testTargetIsMethod() {
    Target target = Trace.class.getAnnotation(Target.class);
    assertNotNull(target);

    boolean supportsMethod = false;
    for (ElementType type : target.value()) {
      if (type == ElementType.METHOD) {
        supportsMethod = true;
        break;
      }
    }

    assertTrue(supportsMethod);
  }

  @Test
  void testAnnotationPresentOnMethod() throws Exception {
    Method method = TraceTest.class.getDeclaredMethod("annotatedMethod");

    assertTrue(method.isAnnotationPresent(Trace.class));
  }

  @Test
  void testAnnotationAbsentWhenNotDeclared() throws Exception {
    Method method = TraceTest.class.getDeclaredMethod("nonAnnotatedMethod");

    assertFalse(method.isAnnotationPresent(Trace.class));
  }

  @Test
  void testAnnotationMetadataAccessible() {
    assertNotNull(Trace.class.getAnnotation(Retention.class));
    assertNotNull(Trace.class.getAnnotation(Target.class));
  }
}
