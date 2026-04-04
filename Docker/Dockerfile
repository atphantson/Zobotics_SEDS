FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_QPA_PLATFORM=xcb
ENV LIBGL_ALWAYS_SOFTWARE=1

RUN apt update && apt install -y \
    wget \
    ca-certificates \
    dbus-x11 \
    libxcb1 \
    libxcb-xinerama0 \
    libx11-xcb1 \
    libxrender1 \
    libxi6 \
    libxkbcommon-x11-0 \
    libglu1-mesa \
    libgl1-mesa-glx \
    libegl1 \
    libdbus-1-3 \
    libfontconfig1 \
    libfreetype6 \
    && rm -rf /var/lib/apt/lists/*

# Télécharger et installer le .deb
RUN wget -O /tmp/ft.deb \
    https://github.com/Kotakku/FT_SCServo_Debug_Qt/releases/download/v1.1.0/ft-scservo-debug-qt_1.1.0_amd64.deb \
    && apt update \
    && apt install -y /tmp/ft.deb \
    && rm /tmp/ft.deb \
    && rm -rf /var/lib/apt/lists/*

CMD ["/usr/bin/FT_SCServo_Debug_Qt"]
