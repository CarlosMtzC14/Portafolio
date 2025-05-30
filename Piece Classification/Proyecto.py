import time
import numpy as np
import cv2
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print('Iniciando sistema de clasificación de monedas...')
client = RemoteAPIClient()
sim = client.require('sim')

# Obtener objetos
visionSensor = sim.getObject('/visionSensor')
joints = [
    sim.getObject('/Cuboid/Cuboid[0]/Prismatic_joint'),  # 1 pesos
    sim.getObject('/Cuboid/Cuboid[1]/Prismatic_joint'),  # 2 pesos
    sim.getObject('/Cuboid/Cuboid[2]/Prismatic_joint'),  # 5 pesos
    sim.getObject('/Cuboid/Cuboid[3]/Prismatic_joint')   # 10 pesos
]

# Activar motores
for joint in joints:
    sim.setObjectInt32Param(joint, sim.jointintparam_motor_enabled, 1)
    sim.setObjectInt32Param(joint, sim.jointintparam_ctrl_enabled, 1)

sim.setStepping(True)
sim.startSimulation()

# Rango de áreas por tipo de moneda
rangos_monedas = {
    (500, 1000): 1,     # 1 peso
    (8000, 9000): 2,     # 2 pesos
    (9000, 9600): 5,     # 5 pesos
    (9600, 10000): 10    # 10 pesos
}

def detectar_moneda(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        for (min_area, max_area), valor in rangos_monedas.items():
            if min_area <= area < max_area:
                return valor, morph
    return None, morph

while True:
    img, [resX, resY] = sim.getVisionSensorImg(visionSensor)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_RGB2BGR), 0)

    valor_detectado, mask = detectar_moneda(img)

    if valor_detectado is not None:
        print(f"Moneda detectada de {valor_detectado} pesos")

        if valor_detectado == 1:
            sim.setJointTargetPosition(joints[0], 0.2)
        elif valor_detectado == 2:
            sim.setJointTargetPosition(joints[1], 0.2)
        elif valor_detectado == 5:
            sim.setJointTargetPosition(joints[2], 0.2)
        elif valor_detectado == 10:
            sim.setJointTargetPosition(joints[3], 0.2)

        for _ in range(5):
            sim.step()
            time.sleep(0.5)

        for joint in joints:
            sim.setJointTargetPosition(joint, 0.0)

        for _ in range(5):
            sim.step()
            time.sleep(0.05)

    cv2.imshow("Sensor de vision", img)
    cv2.imshow("Mascara procesada", mask)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break

    sim.step()

sim.stopSimulation()
while sim.getSimulationState() != sim.simulation_stopped:
    time.sleep(0.1)

cv2.destroyAllWindows()
