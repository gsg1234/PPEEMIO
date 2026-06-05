from emioapi import EmioMotors, EmioCamera
import time
import numpy as np
import random as rnd

def obtener_gdl_bloqueados_con_nombres(restricciones, numeracion_nodos, gdl_por_nodo=3):
    mapa_gdl = {
        "x": 0,
        "y": 1,
        "tita": 2
    }

    gdl_bloqueados = []

    for nombre_nodo, restriccion in restricciones.items():
        nodo = numeracion_nodos[nombre_nodo]

        for direccion, esta_restringido in restriccion.items():
            if esta_restringido:
                gdl_global = nodo * gdl_por_nodo + mapa_gdl[direccion]
                gdl_bloqueados.append(gdl_global)

    return gdl_bloqueados

def main():
    a = np.zeros(10)
    b = np.ones(10)

    c = np.concatenate((a, b))

    print(c.shape)
    print(c)

if __name__ == "__main__":
    main()