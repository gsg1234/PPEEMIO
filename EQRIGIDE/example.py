import tkinter as tk
from tkinter import messagebox
import math
from emioapi import EmioAPI

class PPE_EMIO_GUI:
    def __init__(self, master):
        self.master = master
        self.master.title("Control de Motores - PPE_EMIO_GUI")
        self.master.geometry("550x350")
        
        # Interceptar el evento de cierre de ventana para desconectar el robot de forma segura
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Variables de Tkinter (en GRADOS para la interfaz)
        self.angles_write = [tk.DoubleVar(value=0.0) for _ in range(4)]
        self.angles_read = [tk.DoubleVar(value=0.0) for _ in range(4)]
        
        # Inicialización de la API Emio
        self.emio = EmioAPI(multiprocess_camera=False)
        self.connected = False
        
        self.create_widgets()
        self.connect_device()

    def connect_device(self):
        """Intenta conectar al dispositivo Emio al iniciar la GUI."""
        print("Conectando al dispositivo Emio...")
        if self.emio.connectToEmioDevice():
            print("Conectado exitosamente al dispositivo Emio.")
            self.emio.printStatus()
            self.connected = True
            self.status_label.config(text="Estado: Conectado", fg="green")
            
            # Iniciar el bucle de actualización visual en tiempo real
            self.update_readings()
        else:
            print("Fallo al conectar al dispositivo Emio.")
            self.status_label.config(text="Estado: Fallo de Conexión", fg="red")
            messagebox.showwarning("Advertencia", "No se pudo conectar al dispositivo Emio. Revisa la conexión.")

    def create_widgets(self):
        """Crea todos los elementos de la ventana."""
        # Etiqueta de estado
        self.status_label = tk.Label(self.master, text="Estado: Conectando...", font=("Arial", 10, "italic"))
        self.status_label.grid(row=0, column=0, columnspan=3, pady=10)
        
        # Encabezados
        tk.Label(self.master, text="Motor", font=("Arial", 10, "bold")).grid(row=1, column=0, padx=10, pady=5)
        tk.Label(self.master, text="Modificar Ángulo (°)", font=("Arial", 10, "bold")).grid(row=1, column=1, padx=10, pady=5)
        tk.Label(self.master, text="Lectura Real (°)", font=("Arial", 10, "bold")).grid(row=1, column=2, padx=10, pady=5)

        # Generar controles para los 4 motores
        for i in range(4):
            tk.Label(self.master, text=f"Motor {i+1}").grid(row=i+2, column=0, padx=10, pady=10)
            
            # Sliders configurados de -180 a 180 grados
            slider = tk.Scale(self.master, from_=-180, to=180, orient="horizontal", length=250,
                              variable=self.angles_write[i],
                              command=self.send_angles) # Envía los datos al mover el slider
            slider.grid(row=i+2, column=1, padx=10, pady=5)
            
            # Etiqueta para lectura
            read_label = tk.Label(self.master, textvariable=self.angles_read[i], 
                                  width=10, bg="white", relief="sunken", font=("Arial", 10))
            read_label.grid(row=i+2, column=2, padx=10, pady=5)

    def send_angles(self, _=None):
        """
        Método de ESCRITURA: Toma los 4 valores de la GUI (grados), 
        los convierte a radianes y los envía a la API de Emio.
        """
        if not self.connected:
            return
            
        try:
            # Leer los 4 sliders y convertir a radianes
            target_angles_rad = [math.radians(var.get()) for var in self.angles_write]
            
            # Escribir la lista completa de ángulos al dispositivo
            self.emio.motors.angles = target_angles_rad
            
        except Exception as e:
            print(f"Error al enviar comandos: {e}")

    def update_readings(self):
        """
        Método de LECTURA: Lee los 4 ángulos del hardware en tiempo real (radianes), 
        los convierte a grados y los muestra en la GUI.
        """
        if self.connected:
            try:
                # Leer los ángulos reales desde la API
                current_angles_rad = self.emio.motors.angles
                
                # Actualizar las variables de la interfaz
                for i in range(4):
                    # Convertir de radianes a grados
                    angulo_grados = math.degrees(current_angles_rad[i])
                    self.angles_read[i].set(round(angulo_grados, 2))
                    
            except Exception as e:
                print(f"Error al leer motores: {e}")

        # Programar la próxima ejecución en 100 ms (Actualización a 10Hz)
        self.master.after(100, self.update_readings)

    def on_closing(self):
        """Ruta de desconexión segura al presionar la 'X' de la ventana."""
        if self.connected:
            print("Cerrando la aplicación y desconectando el dispositivo Emio...")
            try:
                self.emio.disconnect()
            except Exception as e:
                print(f"Error al intentar desconectar: {e}")
        
        # Destruir la ventana
        self.master.destroy()

# ==========================================
# Ejecución Principal
# ==========================================
if __name__ == "__main__":
    root = tk.Tk()
    app = PPE_EMIO_GUI(root)
    root.mainloop()