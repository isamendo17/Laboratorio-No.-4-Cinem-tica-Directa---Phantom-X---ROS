# Laboratorio #4 Cinemática Directa - Phantom X - ROS

## Integrantes:
- Isabella Mendoza Cáceres
- Andrés Santiago Cañón Porras

## Descripción detallada de la solución planteada

## Diagrama de flujo de acciones del robot unsando la herramieenta Mermaid

## Plano de planta de la ubicación de cada uno de los elementos

## Descripción de las funciones utilizadas
A continuación se presenta en una tabla las funciones utilizadas en el código para desarrollar el laboratorio.

| Función                             | Descripción                                                                 |
|-------------------------------------|-----------------------------------------------------------------------------|
| `__init__(self)`                    | Inicializa el nodo ROS, configura Dynamixel y construye la interfaz gráfica. |
| `create_title(self)`                | Muestra el título superior de la interfaz.                                  |
| `create_button_row(self)`           | Crea los botones horizontales para enviar posiciones predefinidas.          |
| `create_joint_labels(self)`         | Muestra etiquetas para las posiciones actuales de cada articulación.        |
| `create_footer(self)`               | Añade una línea de crédito en la parte inferior.                            |
| `enviar_posicion(self, nombre)`     | Llama a `move_to_position` con una posición del diccionario.                |
| `move_to_position(self, pos)`       | Escribe la posición, velocidad y torque a cada motor; luego actualiza la GUI. |
| `read_current_positions(self)`      | Lee desde los motores las posiciones actuales en bits.                      |
| `update_joint_positions_display(self, pos)` | Muestra los valores convertidos en grados en la GUI.               |
| `convert_bits_to_degrees(self, bits)`| Convierte valores de 0–1023 a grados (±150° centrado en 512).              |
| `run(self)`                         | Ejecuta el bucle principal de la ventana tkinter.                           |
| `main()`                            | Inicializa y ejecuta el nodo ROS junto con la interfaz.                     |

## Código del script utilizado para el desarrollo de la práctica
El código utilizado por el pincher se encuentra actualmente subido como anexo del GitHub. No obstante, también se muestra a continuación explicado.

<code>```
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time
import tkinter as tk

# Direcciones de registro en el AX-12A
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Declaración de parámetros ROS 2 con valores por defecto
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [512, 512, 512, 512, 512])
        self.declare_parameter('moving_speed', 100) # 0–1023 
        self.declare_parameter('torque_limit', 1000) # 0–1023 
        self.declare_parameter('delay', 2.0)

        # Obtiene los parámetros declarados
        port_name      = self.get_parameter('port').value
        baudrate       = self.get_parameter('baudrate').value
        self.dxl_ids   = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        self.moving_speed = self.get_parameter('moving_speed').value
        self.torque_limit = self.get_parameter('torque_limit').value
        self.delay_seconds = self.get_parameter('delay').value

        # Verifica que haya una posición para cada ID
        if len(goal_positions) != len(self.dxl_ids):
            self.get_logger().error('Error: la longitud de goal_positions no coincide con dxl_ids')
            rclpy.shutdown()
            return
        
         # Inicializar comunicación
        self.port = PortHandler(port_name)
        self.port.openPort()
        self.port.setBaudRate(baudrate)
        self.packet = PacketHandler(1.0)

        # Crea la ventana de interfaz gráfica
        self.window = tk.Tk()
        self.window.title("Robotcito :p")
        self.window.geometry("600x450")
        self.window.configure(bg="#f0f4f7")

        # Diccionario de posiciones predefinidas para el robot
        self.posiciones = {
            "Mov 1": [512, 512, 512, 512, 512],
            "Mov 2": [597, 597, 580, 444, 512],
            "Mov 3": [392, 631, 410, 614, 512],
            "Mov 4": [802, 444, 700, 597, 512],
            "Mov 5": [785, 392, 700, 358, 512]
        }

        self.joint_labels = []

        # Crea elementos visuales
        self.create_title()
        self.create_button_row()  # nuevo layout horizontal
        self.create_joint_labels()
        self.create_footer()

    def create_title(self):
        tk.Label(self.window, text="Interfaz de Control del Robot", 
                 font=("Helvetica", 16, "bold"), bg="#f0f4f7", fg="#333").pack(pady=10)

    def create_button_row(self):
        boton_frame = tk.Frame(self.window, bg="#f0f4f7")
        boton_frame.pack(pady=10)

        # Crea un botón para cada movimiento predefinido
        for nombre in self.posiciones:
            boton = tk.Button(boton_frame, text=nombre, width=12, height=2,
                              bg="#4a90e2", fg="white", font=("Arial", 10, "bold"),
                              command=lambda n=nombre: self.enviar_posicion(n))
            boton.pack(side=tk.LEFT, padx=5)

        # Botón salir 
        tk.Button(self.window, text="Salir", command=self.window.quit,
                  bg="#e74c3c", fg="white", font=("Arial", 10, "bold"),
                  width=30, height=2).pack(pady=15)
    
    # Creación de etiquetas
    def create_joint_labels(self):
        tk.Label(self.window, text="Ángulos reales de las articulaciones:",
                 font=("Arial", 12, "bold"), bg="#f0f4f7", fg="#333").pack(pady=10)

        for i in range(len(self.dxl_ids)):
            etiqueta = tk.Label(self.window, text=f'Articulación {i+1} = ---°',
                                font=("Arial", 10), bg="#f0f4f7", fg="#000")
            etiqueta.pack(pady=2)
            self.joint_labels.append(etiqueta)

    # Pie de página con los nombres de los integrantes
    def create_footer(self):
        tk.Label(self.window, text="Diseñado por Santi & Isa ❤️",
                 font=("Arial", 10, "italic"), bg="#f0f4f7", fg="#666").pack(pady=30)
    
    # Llama a la función que envía posiciones a motores
    def enviar_posicion(self, nombre_posicion):
        self.move_to_position(self.posiciones[nombre_posicion])

    # Envía comandos de posición y velocidad a cada motor
    def move_to_position(self, goal_positions):
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, self.moving_speed)
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)

        # Espera y luego lee las posiciones actual
        time.sleep(self.delay_seconds)
        current_positions = self.read_current_positions()
        self.update_joint_positions_display(current_positions)

    # Lee las posiciones actuales de todos los motores
    def read_current_positions(self):
        current_positions = []
        for dxl_id in self.dxl_ids:
            pos, _, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
            current_positions.append(pos)
        return current_positions
    
    # Actualiza las etiquetas con las posiciones en grados
    def update_joint_positions_display(self, positions):
        for i, pos in enumerate(positions):
            grados = self.convert_bits_to_degrees(pos)
            self.joint_labels[i].config(text=f'Articulación {i+1} = {grados:.2f}°')

    # Conversión de bits a grados
    def convert_bits_to_degrees(self, bits):
        return ((bits - 512) / 512) * 150
    
    # Inicia la interfaz gráfica
    def run(self):
        self.window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    pincher_controller = PincherController()
    pincher_controller.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

</code> 

## Video del brazo alcanzando cada posición solicitada

## Vídeo demostración de uso de la interfaz de usuario
Los videos debe comenzar con la introducción oficial del laboratorio LabSIR Intro LabSIR.
