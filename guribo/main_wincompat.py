import pybullet as p
import pybullet_data
import time
import threading
import queue
import math
import paho.mqtt.client as mqtt
import os
import subprocess

import pybullet_data
import subprocess

def caminho_curto(caminho_original):
    try:
        comando = f'for %I in ("{caminho_original}") do @echo %~sI'
        resultado = subprocess.check_output(comando, shell=True, text=True)
        return resultado.strip()
    except subprocess.CalledProcessError:
        print("Erro ao obter caminho curto.")
        return caminho_original  # fallback

# Use caminho curto para o pybullet_data
caminho_pybullet_data = pybullet_data.getDataPath()
caminho_pybullet_data_curto = caminho_curto(caminho_pybullet_data)
print(f"[INFO] Caminho pybullet_data curto: {caminho_pybullet_data_curto}")

# ================================
# MQTT Configuration and Connection
# ================================
mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
mqtt_topic = "husky/checkin"

try:
    mqtt_client.connect("localhost", 1883, 60)
    mqtt_connected = True
    print("[MQTT] Conectado com sucesso ao broker.")
    print(pybullet_data.getDataPath())
except Exception as e:
    print(f"[MQTT] Não foi possível conectar ao broker: {e}")
    mqtt_connected = False

# ================================
# Queue para exportação de dados
# ================================
results_queue = queue.Queue()

# ================================
# Lista de Pontos de Inspeção
# ================================
inspection_points = [
    {"id": "Base1", "pos": [2, 0]},
    {"id": "Base2", "pos": [2, 2]},
    {"id": "Base3", "pos": [0, 2]},
    {"id": "Base4", "pos": [0, 0]}
]

CHECKIN_RADIUS = 0.3
left_wheels = [2, 4]
right_wheels = [3, 5]

# ================================
# Ambiente PyBullet
# ================================
def setup_environment():
    client_id = p.connect(p.GUI)
    if client_id < 0:
        print("[ERROR] Não foi possível iniciar o PyBullet em modo GUI.")
        exit(1)
    p.setAdditionalSearchPath(caminho_pybullet_data_curto)
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    husky = p.loadURDF("husky/husky.urdf", basePosition=[0, 0, 0.1])
    return husky

# ================================
# Movimento e Navegação
# ================================
def get_robot_position(husky_id):
    pos, _ = p.getBasePositionAndOrientation(husky_id)
    return pos[0], pos[1]

def compute_control(target_x, target_y, current_x, current_y):
    dx = target_x - current_x
    dy = target_y - current_y
    angle = math.atan2(dy, dx)
    linear_speed = 5.0
    return linear_speed

def stop_robot(husky_id):
    for j in left_wheels + right_wheels:
        p.setJointMotorControl2(husky_id, j, p.VELOCITY_CONTROL, targetVelocity=0, force=100)

def move_robot(husky_id, velocity):
    for j in left_wheels:
        p.setJointMotorControl2(husky_id, j, p.VELOCITY_CONTROL, targetVelocity=velocity, force=100)
    for j in right_wheels:
        p.setJointMotorControl2(husky_id, j, p.VELOCITY_CONTROL, targetVelocity=velocity, force=100)

# ================================
# Exportação via MQTT
# ================================
def export_data(message):
    if not mqtt_connected:
        print("[MQTT] Broker não conectado. Mensagem ignorada.")
        return
    point_id = message[0]
    pos = message[1]
    data = {
        "point_id": point_id,
        "x": float(pos[0]),
        "y": float(pos[1]),
        "timestamp": time.time()
    }
    try:
        mqtt_client.publish(mqtt_topic, str(data))
        print(f"[MQTT] Enviado: {data}")
    except Exception as e:
        print(f"[MQTT] Erro ao enviar dados: {e}")

def export_thread():
    while True:
        try:
            if not results_queue.empty():
                message = results_queue.get_nowait()
                export_data(message)
                results_queue.task_done()
                time.sleep(0.1)
        except queue.Empty:
            continue
        except Exception as e:
            print(f"[Export Error] {e}")
            break

# ================================
# Operação Principal (thread)
# ================================
def operation_thread():
    husky = setup_environment()
    print("[INFO] Iniciando inspeção automática...")

    for point in inspection_points:
        target_x, target_y = point["pos"]
        reached = False

        while not reached:
            x, y = get_robot_position(husky)
            dist = math.hypot(target_x - x, target_y - y)

            if dist < CHECKIN_RADIUS:
                stop_robot(husky)
                results_queue.put((point["id"], (x, y)))
                print(f"[CHECK-IN] {point['id']}")
                reached = True
                time.sleep(1)
            else:
                velocity = compute_control(target_x, target_y, x, y)
                move_robot(husky, velocity)

            p.stepSimulation()
            time.sleep(1 / 240.0)

    print("[INFO] Rota concluída. Desconectando.")
    stop_robot(husky)
    time.sleep(1)
    p.disconnect()
    input("Pressione Enter para sair...")  # Evita que feche automaticamente

# ================================
# Main
# ================================
def main():
    op_thread = threading.Thread(target=operation_thread)
    exp_thread = threading.Thread(target=export_thread, daemon=True)
    op_thread.start()
    exp_thread.start()
    op_thread.join()

if __name__ == "__main__":
    main()