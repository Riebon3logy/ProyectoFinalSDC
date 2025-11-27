import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import re
from scipy.signal import savgol_filter

# Variable global para mantener la conexion serial
ser = None
# Variables para almacenar datos de la grafica
distance_data = deque(maxlen=100)  # Últimos 100 puntos
time_data = deque(maxlen=100)
time_counter = 0
is_reading = False
read_thread = None
current_distance = 0.0
setpoint = 22.0  # Valor de referencia inicial


# Establece la comunicacion serial con el puerto seleccionado
def connect_to_port(port):
    global ser, time_counter

    # Reiniciar contador de tiempo
    time_counter = 0
    distance_data.clear()
    time_data.clear()

    # Cerrar conexion previa si existe
    if ser and ser.is_open:
        ser.close()
        print(f"Closed previous connection")

    try:
        # Intenta abrir el puerto serial
        ser = serial.Serial(
            port=port,
            baudrate=9600,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        print(f"Connected to {port}")
        messagebox.showinfo("Conexion exitosa", f"Conectado a {port}")
        start_reading()
        return True
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        messagebox.showerror("Error de conexion", f"No se pudo conectar a {port}\n{e}")
        return False


# Imprime el puerto seleccionado y establece la conexion
def on_select(selection):
    selected_port = selection
    print(f"Selected port: {selected_port}")

    if selected_port != "No Ports Found":
        connect_to_port(selected_port)


# Funcion para actualizar el display de distancia en la GUI
def update_distance_display(distance_value):
    distance_display.config(state='normal')
    distance_display.delete(0, tk.END)
    distance_display.insert(0, f"{distance_value:.1f}")
    distance_display.config(state='readonly')


# Funcion para leer datos del serial en un hilo separado
def read_serial_data():
    global time_counter, is_reading, current_distance

    while is_reading and ser and ser.is_open:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"Received: {line}")  # Debug: mostrar todas las líneas recibidas

                # Buscar líneas que contengan "Distancia IR:" (corregido)
                if "Distancia IR:" in line:
                    try:
                        # Extraer el valor de distancia usando regex
                        # Formato esperado: "Distancia IR: 20.50 cm"
                        match = re.search(r'Distancia IR:\s*([\d.]+)', line)
                        if match:
                            distance_value = float(match.group(1))
                            current_distance = distance_value
                            distance_data.append(distance_value)
                            time_data.append(time_counter)
                            time_counter += 1

                            # Actualizar el display en la GUI
                            root.after(0, update_distance_display, distance_value)

                            print(f"Graficando - Distance: {distance_value} cm, Time: {time_counter}")
                    except (ValueError, AttributeError) as e:
                        print(f"Error parsing distance: {e}")
        except Exception as e:
            print(f"Error reading serial: {e}")


# Iniciar lectura de datos
def start_reading():
    global is_reading, read_thread

    if not is_reading:
        is_reading = True
        read_thread = threading.Thread(target=read_serial_data, daemon=True)
        read_thread.start()
        print("Started reading thread")


# Detener lectura de datos
def stop_reading():
    global is_reading
    is_reading = False
    print("Stopped reading thread")


# Funcion para enviar los valores PID al Arduino
def send_pid_values():
    global ser

    if not ser or not ser.is_open:
        messagebox.showerror("Error", "No hay conexion serial establecida")
        return

    try:
        kp = float(kp_entry.get())
        ki = float(ki_entry.get())
        kd = float(kd_entry.get())
        current_setpoint = float(setpoint_entry.get())

        # Enviar junto kp,ki,kd,setpoint
        message = f"{kp},{ki},{kd},{current_setpoint}\n"

        ser.write(message.encode())
        print(f"Sent: Kp={kp}, Ki={ki}, Kd={kd}, Setpoint={current_setpoint}")
        messagebox.showinfo("Éxito", f"Valores enviados:\nKp={kp}\nKi={ki}\nKd={kd}\nSetpoint={current_setpoint}")

    except ValueError:
        messagebox.showerror("Error", "Por favor ingresa valores numéricos validos")
    except serial.SerialException as e:
        messagebox.showerror("Error de comunicacion", f"Error al enviar datos: {e}")

# Funcion para enviar la referencia al Arduino
def send_setpoint():
    global ser, setpoint

    if not ser or not ser.is_open:
        messagebox.showerror("Error", "No hay conexion serial establecida")
        return

    try:
        # Obtener el valor de referencia del textbox
        new_setpoint = float(setpoint_entry.get())

        # Validar rango razonable (opcional)
        if new_setpoint < 8 or new_setpoint > 40:
            messagebox.showwarning("Advertencia", "La referencia debe estar entre 8 y 40 cm")
            return

        setpoint = new_setpoint

        # Formato del mensaje a enviar (ajusta según tu protocolo Arduino)
        message = f"S{setpoint}\n"  # Ejemplo: "S22.0\n"

        # Enviar por serial
        ser.write(message.encode())
        print(f"Sent Setpoint: {setpoint} cm")
        messagebox.showinfo("Éxito", f"Referencia enviada: {setpoint} cm")

    except ValueError:
        messagebox.showerror("Error", "Por favor ingresa un valor numérico válido")
    except serial.SerialException as e:
        messagebox.showerror("Error de comunicacion", f"Error al enviar datos: {e}")


# Funcion para actualizar la grafica
def update_plot(frame):
    if len(distance_data) > 0:
        ax.clear()

        # Aplicar filtro Savitzky-Golay (requiere al menos 5 puntos)
        if len(distance_data) >= 5:
            smoothed_distance = savgol_filter(list(distance_data), window_length=5, polyorder=2)
        else:
            smoothed_distance = list(distance_data)

        ax.plot(list(time_data), smoothed_distance, 'b-', linewidth=2, label='Distancia actual')
        ax.axhline(y=setpoint, color='r', linestyle='--', linewidth=2, label=f'Setpoint ({setpoint} cm)')
        ax.set_xlabel('Tiempo (mS)', fontsize=10)
        ax.set_ylabel('Distancia (cm)', fontsize=10)
        ax.set_title('Distancia en Tiempo Real', fontsize=12, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)

        # ===== LÍMITES FIJOS =====
        # Establecer límites fijos en el eje Y (ajusta según tu rango esperado)
        ax.set_ylim(0, 50)  # De 0 a 50 cm

        # Mostrar solo los últimos 100 puntos en el eje X
        if len(time_data) > 0:
            ax.set_xlim(max(0, time_data[0]), time_data[-1] + 1)


# Listar los puertos seriales
def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]


# Funcion para cerrar correctamente al salir
def on_closing():
    global ser
    stop_reading()
    if ser and ser.is_open:
        ser.close()
        print("Serial connection closed")
    root.destroy()


# Instancia de la Tk
root = tk.Tk()
root.title("Proyecto final - Balancin aplicando PID")
root.geometry("1000x700")

# Get the list of available ports
available_ports = list_serial_ports()

if not available_ports:
    available_ports = ["No Ports Found"]
    print("No serial ports found.")

# Mantener el puerto serial
selected_port_var = tk.StringVar(root)
selected_port_var.set(available_ports[0])

# Frame principal dividido en dos partes
main_frame = ttk.Frame(root)
main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

# Frame izquierdo para controles
left_frame = ttk.Frame(main_frame, padding="10")
left_frame.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 5))

# Frame derecho para la grafica
right_frame = ttk.Frame(main_frame, padding="10")
right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))

# ===== SECCION DE CONEXION (Frame izquierdo) =====
label = ttk.Label(left_frame, text="Selecciona el puerto serial:", font=("Arial", 10, "bold"))
label.pack(pady=(0, 10))

# Creacion y colocacion del combobox
port_dropdown = ttk.Combobox(left_frame, textvariable=selected_port_var, values=available_ports, state="readonly",
                             width=20)
port_dropdown.pack(pady=10)

# Frame para botones de conexion
connection_frame = ttk.Frame(left_frame)
connection_frame.pack(pady=10)

# Boton para conectar manualmente
connect_button = ttk.Button(connection_frame, text="Conectar", command=lambda: on_select(selected_port_var.get()))
connect_button.grid(row=0, column=0, padx=5, pady=5)


# Boton para refrescar puertos
def refresh_ports():
    new_ports = list_serial_ports()
    if not new_ports:
        new_ports = ["No Ports Found"]
    port_dropdown['values'] = new_ports
    if new_ports:
        selected_port_var.set(new_ports[0])
    print("Ports refreshed")


refresh_button = ttk.Button(connection_frame, text="Refrescar", command=refresh_ports)
refresh_button.grid(row=0, column=1, padx=5, pady=5)

# Separador
separator = ttk.Separator(left_frame, orient='horizontal')
separator.pack(fill='x', pady=10)

# ===== SECCION DE DISTANCIA ACTUAL =====
distance_label = ttk.Label(left_frame, text="Distancia real (cm):", font=("Arial", 10, "bold"))
distance_label.pack(pady=(0, 10))

# Frame para el display de distancia
distance_frame = ttk.Frame(left_frame)
distance_frame.pack(pady=10)

# Textbox de solo lectura para mostrar la distancia
distance_display = ttk.Entry(distance_frame, width=15, font=("Arial", 14, "bold"),
                             justify='center', state='readonly')
distance_display.pack(padx=5, pady=5)

# Insertar valor inicial
distance_display.config(state='normal')
distance_display.insert(0, "0.0")
distance_display.config(state='readonly')

# Separador
separator1_5 = ttk.Separator(left_frame, orient='horizontal')
separator1_5.pack(fill='x', pady=10)

# ===== SECCION DE REFERENCIA (SETPOINT) =====
setpoint_label = ttk.Label(left_frame, text="Referencia (Setpoint):", font=("Arial", 10, "bold"))
setpoint_label.pack(pady=(0, 5))

# Label informativo
setpoint_info = ttk.Label(left_frame, text="(22 cm = centro del balancín)",
                          font=("Arial", 8, "italic"), foreground="gray")
setpoint_info.pack(pady=(0, 10))

# Frame para el setpoint
setpoint_frame = ttk.Frame(left_frame)
setpoint_frame.pack(pady=10)

# Entry para setpoint
setpoint_entry = ttk.Entry(setpoint_frame, width=15, justify='center')
setpoint_entry.pack(padx=5, pady=5)
setpoint_entry.insert(0, "22.0")

# Botón para enviar referencia
send_setpoint_button = ttk.Button(left_frame, text="Enviar Referencia", command=send_setpoint)
send_setpoint_button.pack(pady=10)

# Separador
separator1_75 = ttk.Separator(left_frame, orient='horizontal')
separator1_75.pack(fill='x', pady=10)

# ===== SECCION DE PARAMETROS PID =====
pid_label = ttk.Label(left_frame, text="Parametros PID:", font=("Arial", 10, "bold"))
pid_label.pack(pady=(0, 10))

# Frame para los parametros PID
pid_frame = ttk.Frame(left_frame)
pid_frame.pack(pady=10)

# Kp
kp_label = ttk.Label(pid_frame, text="Kp:")
kp_label.grid(row=0, column=0, padx=5, pady=5, sticky='e')
kp_entry = ttk.Entry(pid_frame, width=15)
kp_entry.grid(row=0, column=1, padx=5, pady=5)
kp_entry.insert(0, "7")

# Ki
ki_label = ttk.Label(pid_frame, text="Ki:")
ki_label.grid(row=1, column=0, padx=5, pady=5, sticky='e')
ki_entry = ttk.Entry(pid_frame, width=15)
ki_entry.grid(row=1, column=1, padx=5, pady=5)
ki_entry.insert(0, "0.155")

# Kd
kd_label = ttk.Label(pid_frame, text="Kd:")
kd_label.grid(row=2, column=0, padx=5, pady=5, sticky='e')
kd_entry = ttk.Entry(pid_frame, width=15)
kd_entry.grid(row=2, column=1, padx=5, pady=5)
kd_entry.insert(0, "22.22")

# Boton para enviar valores PID
send_button = ttk.Button(left_frame, text="Enviar Valores PID", command=send_pid_values)
send_button.pack(pady=20)

# Separador
separator2 = ttk.Separator(left_frame, orient='horizontal')
separator2.pack(fill='x', pady=10)

# ===== SECCION DE GRAFICA (Frame derecho) =====
graph_label = ttk.Label(right_frame, text="Grafica de Distancia", font=("Arial", 10, "bold"))
graph_label.pack(pady=(0, 10))

# Crear la figura de matplotlib
fig, ax = plt.subplots(figsize=(7, 5))
canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Iniciar animacion de la grafica
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

# Vincular el evento de seleccion
port_dropdown.bind("<<ComboboxSelected>>", lambda event: on_select(selected_port_var.get()))

# Protocolo de cierre
root.protocol("WM_DELETE_WINDOW", on_closing)

# Ejecutar la GUI
root.mainloop()