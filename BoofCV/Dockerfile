# Use JDK 17
FROM eclipse-temurin:17-jdk-jammy

# Install GUI/X11 dependencies
RUN apt-get update && apt-get install -y \
    git \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    libglu1-mesa \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Clone SNAPSHOT branch
RUN git clone --depth 1 --branch SNAPSHOT --recursive https://github.com/lessthanoptimal/BoofCV.git .

# Build sequence
RUN ./gradlew autogenerate
RUN ./gradlew applicationsJar

# Debug/Find: This search is broader to ensure we catch the JAR wherever Gradle put it
RUN find . -name "applications*.jar" -exec cp {} /app/boofcv_apps.jar \;

RUN mkdir -p /app/dataset

# Increase Max Heap Size to 8GB for your 61MP Sony images
ENTRYPOINT ["java", "-Xmx8g", "-jar", "/app/boofcv_apps.jar"]