import pybullet as p
import pybullet_data
import time

# Conectar ao simulador
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Carregar plano e robô Husky
plane = p.loadURDF("plane.urdf")
husky = p.loadURDF("husky/husky.urdf", basePosition=[0, 0, 0.1])

# Índices das rodas (obtido com getJointInfo)
wheel_joints = [2, 3, 4, 5]  # Frente esquerda, frente direita, traseira esquerda, traseira direita

# Define velocidade constante para movimento circular (ex: direita mais rápida que esquerda)
left_speed = 5.0
right_speed = 2.0

# Loop da simulação
try:
    while True:
        for joint in [2, 4]:  # Lado esquerdo
            p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,
                                    targetVelocity=left_speed, force=100)

        for joint in [3, 5]:  # Lado direito
            p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,
                                    targetVelocity=right_speed, force=100)

        p.stepSimulation()
        time.sleep(1. / 240.)
except KeyboardInterrupt:
    p.disconnect()
