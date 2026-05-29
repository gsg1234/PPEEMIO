import json
import numpy as np

np.set_printoptions(suppress=True,linewidth=None)

def parse_efforts(data):
    num_nodes = 21
    vector = np.zeros(3 * num_nodes)

    for i in range(num_nodes):
        node_name = f"Node{i}"

        vector[3*i] = data[node_name]["Fx"]
        vector[3*i+1] = data[node_name]["Fy"]
        vector[3*i+2] = data[node_name]["M"]

    return vector


# Ejemplo de uso leyendo desde un archivo JSON
if __name__ == "__main__":
    with open("efforts.json", "r") as file:
        data = json.load(file)

    result_vector = parse_efforts(data)

    print(result_vector)
    print("Longitud del vector:", len(result_vector))
