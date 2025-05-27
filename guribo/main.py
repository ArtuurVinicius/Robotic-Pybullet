import pybullet as p
import pybullet_data
import time

# Iniciar GUI
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Carregar plano e robô
plane = p.loadURDF("plane.urdf")
husky = p.loadURDF("husky/husky.urdf", basePosition=[0, 0, 0.1])

# Índices das rodas
left_wheels = [2, 4]
right_wheels = [3, 5]

# Criar sliders (atuam como botões binários)
forward_slider = p.addUserDebugParameter("↑ Frente", 0, 1, 0)
backward_slider = p.addUserDebugParameter("↓ Ré", 0, 1, 0)
left_slider = p.addUserDebugParameter("← Esquerda", 0, 1, 0)
right_slider = p.addUserDebugParameter("→ Direita", 0, 1, 0)

try:
    while True:
        # Lê valores dos sliders
        forward = p.readUserDebugParameter(forward_slider)
        backward = p.readUserDebugParameter(backward_slider)
        left = p.readUserDebugParameter(left_slider)
        right = p.readUserDebugParameter(right_slider)

        # Inicializa velocidades
        left_speed = 0
        right_speed = 0
        base_speed = 5.0

        # Lógica de movimento
        if forward > 0.5:
            left_speed = base_speed
            right_speed = base_speed
        elif backward > 0.5:
            left_speed = -base_speed
            right_speed = -base_speed
        elif left > 0.5:
            left_speed = -base_speed
            right_speed = base_speed
        elif right > 0.5:
            left_speed = base_speed
            right_speed = -base_speed

        # Enviar comandos para as rodas
        for j in left_wheels:
            p.setJointMotorControl2(husky, j, p.VELOCITY_CONTROL,
                                    targetVelocity=left_speed, force=100)

        for j in right_wheels:
            p.setJointMotorControl2(husky, j, p.VELOCITY_CONTROL,
                                    targetVelocity=right_speed, force=100)

        p.stepSimulation()
        time.sleep(1 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
