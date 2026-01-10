# Use Jammy (Ubuntu 22.04)
FROM eclipse-temurin:17-jre-jammy

# Install base dependencies and specific math libraries for mrcal
RUN apt-get update && apt-get install -y \
    wget \
    libopencv-dev \
    libusb-1.0-0-dev \
    libcamera-dev \
    libv4l-dev \
    gstreamer1.0-plugins-base \
    # Fix for the "libsuitesparseconfig.so.5" error
    libsuitesparseconfig5 \
    libcholmod3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

ARG PV_VERSION=v2025.3.1
RUN wget https://github.com/PhotonVision/photonvision/releases/download/${PV_VERSION}/photonvision-${PV_VERSION}-linuxx64.jar -O photonvision.jar

RUN mkdir photonvision_config

EXPOSE 5800

ENTRYPOINT ["java", "-jar", "photonvision.jar"]