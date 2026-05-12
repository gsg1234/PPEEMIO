import numpy as np

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

    return np.array(gdl_bloqueados)

noeuds_contraintes = {
        "1": 0,
        "2": 20
    }

ddl_bloque = {
    "1": {"x": True, "y": True, "tita": True},
    "2": {"x": True, "y": False,  "tita": True}
}

ddl_bloque = obtener_gdl_bloqueados_con_nombres(ddl_bloque, noeuds_contraintes)

print(ddl_bloque)
print(np.arange(3*21))