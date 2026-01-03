# Multi-stage build for Spring Boot application
# Stage 1: Build
FROM eclipse-temurin:17-jdk-jammy AS builder

WORKDIR /app

# Install dos2unix to fix line endings (Windows CRLF -> Linux LF)
RUN apt-get update && \
    apt-get install -y --no-install-recommends dos2unix && \
    rm -rf /var/lib/apt/lists/*

# Copy Gradle wrapper and configuration files (from root)
COPY gradle/ gradle/
COPY gradlew .
COPY build.gradle .
COPY settings.gradle .

# Copy backend module structure
COPY backend/build.gradle backend/
COPY backend/rcp-app/ backend/rcp-app/
COPY backend/rcp-base/ backend/rcp-base/

# Fix line endings (Windows CRLF -> Linux LF) and make gradlew executable
RUN dos2unix gradlew && \
    chmod +x gradlew

# Verify gradlew is executable and has correct shebang
RUN head -1 gradlew | grep -q "#!/bin/sh" || (echo "Invalid gradlew shebang" && exit 1)

# Build the application (skip tests for faster builds)
# Build backend module specifically
RUN bash gradlew :backend:rcp-app:build -x test --no-daemon

# Stage 2: Runtime
FROM eclipse-temurin:17-jre-jammy

WORKDIR /app

# Install curl for healthcheck
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    rm -rf /var/lib/apt/lists/*

# Create non-root user for security
RUN groupadd -r spring && useradd -r -g spring spring

# Copy the built JAR from builder stage
COPY --from=builder /app/backend/rcp-app/build/libs/turtlebot3-backend.jar app.jar

# Change ownership to non-root user
RUN chown spring:spring app.jar

# Switch to non-root user
USER spring

# Expose application port
EXPOSE 8080

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=40s --retries=3 \
  CMD curl -f http://localhost:8080/actuator/health || exit 1

# Run the application
ENTRYPOINT ["java", "-jar", "app.jar"]
