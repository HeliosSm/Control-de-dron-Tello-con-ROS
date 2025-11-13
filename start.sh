#!/bin/bash
echo "=== INICIANDO SISTEMA TELLO ROS2 ==="

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[1/6] Iniciando tello_driver (COMUNICACIÓN CON EL DRON)..."
ros2 run tello_project tello_driver &
sleep 5  # Espera a que se conecte

echo "[2/6] Iniciando video_viewer..."
ros2 run tello_project video_viewer &

echo "[3/6] Iniciando telemetry_monitor..."
ros2 run tello_project telemetry_monitor &

echo "[4/6] Iniciando battery_failsafe..."
ros2 run tello_project battery_failsafe &

echo "[5/6] Iniciando mission_planner..."
ros2 run tello_project mission_planner &

echo "[6/6] Iniciando object_detector..."
ros2 run tello_project object_detector &

echo "=== TODOS LOS NODOS INICIADOS ==="
echo "Ctrl+C para detener"
wait