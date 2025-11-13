# Dockerfile
FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-opencv python3-pip x11-apps ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir "numpy<2" djitellopy opencv-python

WORKDIR /ros2_ws
COPY . /ros2_ws/

RUN rm -rf build install log && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

COPY start.sh /start.sh
RUN chmod +x /start.sh

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_QPA_PLATFORM=xcb

CMD ["/start.sh"]