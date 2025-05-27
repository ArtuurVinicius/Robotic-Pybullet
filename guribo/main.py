import pybullet as p
import pybullet_data
import time

# Iniciar simulação
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Carregar ambiente
plane = p.loadURDF("plane.urdf")
husky = p.loadURDF("husky/husky.urdf", basePosition=[0, 0, 0.1])

# Rodas esquerda e direita
left_wheels = [2, 4]
right_wheels = [3, 5]

# Velocidade base
base_speed = 5.0

try:
    while True:
        keys = p.getKeyboardEvents()

        left_speed = 0
        right_speed = 0

        if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
            left_speed = base_speed
            right_speed = base_speed
        elif ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
            left_speed = -base_speed
            right_speed = -base_speed
        elif ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
            left_speed = -base_speed
            right_speed = base_speed
        elif ord('k') in keys and keys[ord('k')] & p.KEY_IS_DOWN:
            left_speed = base_speed
            right_speed = -base_speed

        # Atualiza motores
        for j in left_wheels:
            p.setJointMotorControl2(husky, j, p.VELOCITY_CONTROL, targetVelocity=left_speed, force=100)
        for j in right_wheels:
            p.setJointMotorControl2(husky, j, p.VELOCITY_CONTROL, targetVelocity=right_speed, force=100)

        p.stepSimulation()
        time.sleep(1 / 240.0)

except KeyboardInterrupt:
    p.disconnect()
