"""
ROBOT MAESTRO TE3001B Peg-in-Hole Teleoperado
Simulación 3-DOF + Computed Torque + Graficación
Prof. Alberto Muñoz Computational Robotics Lab
Tec de Monterrey, 2026

Ejecutar en la PC MAESTRO:
python3 master_robot.py --slave-ip <IP_ESCLAVO>

Controles:
Flechas arriba/abajo mover efector final en +y/-y
Flechas izq/der mover efector final en -x/+x
Q/E abrir/cerrar pinza
ESC salir
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg') # cambiar a 'Qt5Agg' si TkAgg falla
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import socket
import threading
import time
import json
import argparse

# Deshabilitar TODOS los atajos de teclado de matplotlib para que no roben WASD/QE
for key in [k for k in plt.rcParams if k.startswith('keymap.')]:
    plt.rcParams[key] = []

# PARÁMETROS DEL ROBOT 3R
L1, L2, L3 = 0.35, 0.30, 0.20 # longitudes eslabones [m]
M1, M2, M3 = 1.5, 1.0, 0.5 # masas de eslabones [kg]
G_GRAV = 9.81 # aceleración gravitacional [m/s^2]

# Ganancias del Computed Torque
KP = np.diag([120.0, 100.0, 80.0]) # rigidez articular
KV = np.diag([35.0, 28.0, 20.0]) # amortiguamiento (2*sqrt(KP) aprox.)

# Paso de integración
DT = 0.01 # 100 Hz

# CINEMÁTICA DIRECTA 3R PLANAR
def fk_3r(q):
    """
    Cinemática directa para robot 3R planar.
    Returns: np.array([x3, y3])
    """
    q1, q2, q3 = q
    x1 = L1 * np.cos(q1)
    y1 = L1 * np.sin(q1)
    x2 = x1 + L2 * np.cos(q1 + q2)
    y2 = y1 + L2 * np.sin(q1 + q2)
    x3 = x2 + L3 * np.cos(q1 + q2 + q3)
    y3 = y2 + L3 * np.sin(q1 + q2 + q3)
    return np.array([x3, y3])

def fk_3r_full(q):
    """
    Retorna las posiciones de todas las articulaciones y el EF.
    Returns: Matriz (4x2) con posiciones [base, j1, j2, EF].
    """
    q1, q2, q3 = q
    p0 = np.array([0.0, 0.0])
    p1 = np.array([L1*np.cos(q1), L1*np.sin(q1)])
    p2 = p1 + np.array([L2*np.cos(q1+q2), L2*np.sin(q1+q2)])
    p3 = p2 + np.array([L3*np.cos(q1+q2+q3), L3*np.sin(q1+q2+q3)])
    return np.array([p0, p1, p2, p3])

# JACOBIANO ANALÍTICO 3R PLANAR (J R^{2 3 })
def jacobian_3r(q):
    """
    Jacobiano analítico del robot 3R planar.
    """
    q1, q2, q3 = q
    s1 = np.sin(q1)
    s12 = np.sin(q1 + q2)
    s123 = np.sin(q1 + q2 + q3)
    c1 = np.cos(q1)
    c12 = np.cos(q1 + q2)
    c123 = np.cos(q1 + q2 + q3)
    
    J = np.array([
        [-L1*s1 - L2*s12 - L3*s123, -L2*s12 - L3*s123, -L3*s123],
        [ L1*c1 + L2*c12 + L3*c123,  L2*c12 + L3*c123,  L3*c123]
    ])
    return J

# MODELO DINÁMICO SIMPLIFICADO (punto de masa)
def inertia_matrix(q):
    """
    Matriz de inercia M(q) para robot 3R.
    """
    q1, q2, q3 = q
    c2 = np.cos(q2)
    c3 = np.cos(q3)
    c23 = np.cos(q2 + q3)
    
    m11 = (M1*L1**2 + M2*(L1**2 + L2**2 + 2*L1*L2*c2) +
           M3*(L1**2 + L2**2 + L3**2 + 2*L1*L2*c2 +
               2*L1*L3*c23 + 2*L2*L3*c3))
    m12 = (M2*(L2**2 + L1*L2*c2) +
           M3*(L2**2 + L3**2 + L1*L2*c2 + L1*L3*c23 + 2*L2*L3*c3))
    m13 = M3*(L3**2 + L1*L3*c23 + L2*L3*c3)
    m22 = M2*L2**2 + M3*(L2**2 + L3**2 + 2*L2*L3*c3)
    m23 = M3*(L3**2 + L2*L3*c3)
    m33 = M3 * L3**2
    
    M = np.array([[m11, m12, m13],
                  [m12, m22, m23],
                  [m13, m23, m33]])
    return M

def coriolis_matrix(q, dq):
    """
    Matriz de Coriolis y fuerzas centrífugas C(q, dq).
    """
    eps = 1e-5
    n = len(q)
    M0 = inertia_matrix(q)
    C = np.zeros((n, n))
    
    for k in range(n):
        qp = q.copy(); qp[k] += eps
        qm = q.copy(); qm[k] -= eps
        dM_dk = (inertia_matrix(qp) - inertia_matrix(qm)) / (2*eps)
        C += 0.5 * dM_dk * dq[k]
        
    return C

def gravity_vector(q):
    """
    Vector de par gravitacional g(q) para robot 3R planar.
    """
    q1, q2, q3 = q
    c1 = np.cos(q1)
    c12 = np.cos(q1 + q2)
    c123 = np.cos(q1 + q2 + q3)
    
    g1 = G_GRAV * ((M1+M2+M3)*L1*c1 + (M2+M3)*L2*c12 + M3*L3*c123)
    g2 = G_GRAV * ((M2+M3)*L2*c12 + M3*L3*c123)
    g3 = G_GRAV * M3 * L3 * c123
    return np.array([g1, g2, g3])

# COMPUTED TORQUE CONTROLLER
def computed_torque(q, dq, q_des, dq_des, ddq_des, kp=KP, kv=KV, F_ext=None):
    """
    Ley de control de Computed Torque (cancelación exacta de no-linealidades).
    """
    e = q_des - q
    de = dq_des - dq
    
    a_d = ddq_des + kv @ de + kp @ e
    
    M_mat = inertia_matrix(q)
    C_mat = coriolis_matrix(q, dq)
    g_vec = gravity_vector(q)
    
    tau = M_mat @ a_d + C_mat @ dq + g_vec
    
    if F_ext is not None and np.linalg.norm(F_ext) > 0.01:
        J = jacobian_3r(q)
        tau += J.T @ F_ext
        
    tau = np.clip(tau, -500.0, 500.0)
    return tau, e, de

# INTEGRADOR EULER (dinámica del robot)
def integrate_dynamics(q, dq, tau, dt=DT):
    """
    Integra la dinámica del robot usando Euler explícito.
    """
    M_mat = inertia_matrix(q)
    C_mat = coriolis_matrix(q, dq)
    g_vec = gravity_vector(q)
    
    ddq = np.linalg.solve(M_mat, tau - C_mat @ dq - g_vec)
    
    dq_new = dq + ddq * dt
    q_new = q + dq_new * dt
    
    q_limits = np.array([np.pi/2, 2*np.pi/3, np.pi/2])
    q_new = np.clip(q_new, -q_limits, q_limits)
    dq_new = np.clip(dq_new, -3.0, 3.0)
    return q_new, dq_new

# COMUNICACIÓN DE RED MAESTRO (CLIENTE)
class MasterNetClient:
    """
    Cliente UDP del maestro. Envía posición cartesiana al esclavo
    y recibe fuerzas de contacto para feedback háptico.
    """
    def __init__(self, slave_ip, port_tx=9001, port_rx=9002):
        self.slave_ip = slave_ip
        self.port_tx = port_tx
        self.port_rx = port_rx
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.bind(('', self.port_rx))
        self.sock_rx.settimeout(0.005)
        self.Fe = np.zeros(2)
        self.contact = False
        self.slave_state = "APROXIMACIÓN"
        self.last_recv_time = 0.0
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        
    def send_command(self, xd, gripper=True):
        msg = json.dumps({"xd": xd.tolist(), "gripper": int(gripper)})
        self.sock_tx.sendto(msg.encode(), (self.slave_ip, self.port_tx))
        
    def _recv_loop(self):
        while True:
            try:
                data, _ = self.sock_rx.recvfrom(256)
                parsed = json.loads(data.decode())
                self.Fe = np.array(parsed["Fe"])
                self.contact = bool(parsed["contact"])
                self.slave_state = parsed.get("state", "APROXIMACIÓN")
                self.last_recv_time = time.time()
            except (socket.timeout, json.JSONDecodeError):
                pass

# CLASE PRINCIPAL ROBOT MAESTRO
class MasterRobot:
    """
    Simulador completo del robot maestro
    """
    def __init__(self, slave_ip="127.0.0.1"):
        self.q = np.array([0.4, -0.3, 0.2])
        self.dq = np.zeros(3)
        self.q_des = self.q.copy()
        self.dq_des = np.zeros(3)
        self.ddq_des = np.zeros(3)
        
        self.v_cart = np.zeros(2)
        self.v_step = 1.0
        self.keys_held = set()

        self.net = MasterNetClient(slave_ip)
        
        N = 500
        self.hist_t = np.zeros(N)
        self.hist_q = np.zeros((N, 3))
        self.hist_tau = np.zeros((N, 3))
        self.hist_Fe = np.zeros((N, 2))
        self.hist_x = np.zeros((N, 2))
        self.idx = 0
        self.t = 0.0
        
        self.gripper_open = False
        self.q_des = self.q.copy()
        
    def ik_dls(self, x_des, damp=0.01):
        # Siempre partir de q actual para evitar drift en espacio nulo
        self.q_des = self.q.copy()
        for _ in range(10):
            x_cur = fk_3r(self.q_des)
            e_x = x_des - x_cur
            if np.linalg.norm(e_x) < 1e-4:
                break
            J = jacobian_3r(self.q_des)
            JJT = J @ J.T
            Jp = J.T @ np.linalg.inv(JJT + damp**2 * np.eye(2))
            self.q_des = self.q_des + Jp @ e_x
            
    def step(self):
        v = self.v_step
        vx, vy = 0.0, 0.0
        if 'up' in self.keys_held: vy += v
        if 'down' in self.keys_held: vy -= v
        if 'right' in self.keys_held: vx += v
        if 'left' in self.keys_held: vx -= v
        self.v_cart = np.array([vx, vy])

        x_cur = fk_3r(self.q)
        x_des = x_cur + self.v_cart * DT
        
        self.ik_dls(x_des)
        
        tau, e, de = computed_torque(
            self.q, self.dq, self.q_des, self.dq_des, self.ddq_des,
            F_ext = 0.3 * self.net.Fe
        )
        
        self.q, self.dq = integrate_dynamics(self.q, self.dq, tau)
        
        x_ef = fk_3r(self.q)
        self.net.send_command(x_ef, gripper=not self.gripper_open)
        
        i = self.idx % 500
        self.hist_t[i] = self.t
        self.hist_q[i] = self.q
        self.hist_tau[i] = tau
        self.hist_Fe[i] = self.net.Fe
        self.hist_x[i] = x_ef
        self.idx += 1
        self.t += DT

# GRAFICACIÓN EN TIEMPO REAL
def setup_plots(robot):
    fig = plt.figure(figsize=(14, 10), facecolor='#0a0a1a')
    fig.suptitle('TE3001B Robot Maestro 3R | Peg-in-Hole Teleoperado', color='white', fontsize=14, fontweight='bold', y=0.98)
    
    C = ['#00BFFF', '#FF6B6B', '#69FF47', '#FFD700', '#FF69B4', '#00FFD0']
    bg = '#0d1117'
    grid_c = '#1e2530'
    
    ax_robot = fig.add_subplot(2, 2, 1, facecolor=bg)
    ax_tau = fig.add_subplot(2, 2, 2, facecolor=bg)
    ax_force = fig.add_subplot(2, 2, 3, facecolor=bg)
    ax_q = fig.add_subplot(2, 2, 4, facecolor=bg)
    
    for ax in [ax_robot, ax_tau, ax_force, ax_q]:
        ax.tick_params(colors='#aaa')
        ax.xaxis.label.set_color('#aaa')
        ax.yaxis.label.set_color('#aaa')
        ax.title.set_color('white')
        for spine in ax.spines.values():
            spine.set_edgecolor('#333')
        ax.grid(True, color=grid_c, linestyle='--', alpha=0.5)
        
    # Panel 1: Robot 2D
    ax_robot.set_xlim(-0.9, 0.9)
    ax_robot.set_ylim(-0.9, 0.9)
    ax_robot.set_aspect('equal')
    ax_robot.set_title('Vista Cinemática 3R', fontsize=11)
    ax_robot.set_xlabel('x [m]'); ax_robot.set_ylabel('y [m]')
    link_line, = ax_robot.plot([], [], 'o-', color=C[0], linewidth=3, markersize=8, markerfacecolor=C[1])
    ef_dot, = ax_robot.plot([], [], 's', color=C[2], markersize=12, markerfacecolor=C[3], zorder=5)
    conn_text = ax_robot.text(0.02, 0.96, 'SIN CONEXIÓN', transform=ax_robot.transAxes,
                              color='#FF4444', fontsize=10, fontweight='bold',
                              verticalalignment='top',
                              bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a1a2e', edgecolor='#333', alpha=0.9))
    
    hole_x, hole_y = 0.55, 0.10
    ax_robot.add_patch(plt.Circle((hole_x, hole_y), 0.025, color='#FFD700', alpha=0.6))
    ax_robot.plot(hole_x, hole_y, 'x', color='white', markersize=8, markeredgewidth=2)
    ax_robot.text(hole_x+0.03, hole_y+0.03, 'HOLE', color='#FFD700', fontsize=8)
    
    # Panel 2: Torques articulares
    ax_tau.set_title('Torques Articulares [Nm]', fontsize=11)
    ax_tau.set_xlabel('Tiempo [s]'); ax_tau.set_ylabel('tau [Nm]')
    lines_tau = [ax_tau.plot([], [], color=C[i], linewidth=1.5, label=f'tau {i+1}')[0] for i in range(3)]
    ax_tau.legend(loc='upper right', fontsize=9, facecolor='#1a1a2e', labelcolor='white')
    ax_tau.axhline(y=0, color='#444', linewidth=0.8)
    
    # Panel 3: Fuerzas de contacto (PANEL CLAVE para operación a ciegas)
    ax_force.set_title('FUERZAS DE CONTACTO [N] — GUÍA HÁPTICA', fontsize=12, fontweight='bold')
    ax_force.set_xlabel('Tiempo [s]'); ax_force.set_ylabel('F [N]')
    line_Fx, = ax_force.plot([], [], color=C[4], linewidth=2.5, label='Fx (lateral)')
    line_Fy, = ax_force.plot([], [], color=C[5], linewidth=2.5, label='Fy (inserción)')
    ax_force.axhline(y=0, color='#444', linewidth=0.8)
    ax_force.axhline(y=2.0, color='#FF4444', linewidth=1.2, linestyle='--', label='Umbral contacto')
    ax_force.legend(loc='upper right', fontsize=9, facecolor='#1a1a2e', labelcolor='white')
    # Indicador grande de estado del esclavo (visible a ciegas)
    state_text = ax_force.text(0.5, 0.92, 'APROXIMACIÓN', transform=ax_force.transAxes,
                               color='#888888', fontsize=16, fontweight='bold',
                               ha='center', va='top',
                               bbox=dict(boxstyle='round,pad=0.4', facecolor='#1a1a2e',
                                         edgecolor='#444', alpha=0.95))
    
    # Panel 4: Ángulos articulares
    ax_q.set_title('Ángulos Articulares q [rad]', fontsize=11)
    ax_q.set_xlabel('Tiempo [s]'); ax_q.set_ylabel('q [rad]')
    lines_q = [ax_q.plot([], [], color=C[i], linewidth=1.5, label=f'q{i+1}', linestyle=['solid','dashed','dotted'][i])[0] for i in range(3)]
    ax_q.legend(loc='upper right', fontsize=9, facecolor='#1a1a2e', labelcolor='white')
    
    plt.tight_layout(rect=[0, 0.02, 1, 0.96])
    
    return fig, (ax_robot, ax_tau, ax_force, ax_q), (link_line, ef_dot, conn_text, state_text), lines_tau, (line_Fx, line_Fy), lines_q


def main(slave_ip):
    robot = MasterRobot(slave_ip)
    fig, axes, arm_lines, lines_tau, force_lines, lines_q = setup_plots(robot)
    ax_robot, ax_tau, ax_force, ax_q = axes
    link_line, ef_dot, conn_text, state_text = arm_lines
    line_Fx, line_Fy = force_lines
    
    running = [True]
    def sim_loop():
        while running[0]:
            robot.step()
            time.sleep(DT)
    sim_thread = threading.Thread(target=sim_loop, daemon=True)
    sim_thread.start()
    
    def animate(frame):
        n = min(robot.idx, 500)
        if n == 0: return []
        
        i0 = robot.idx % 500
        idx_range = np.arange(i0, i0 + n) % 500
        t = robot.hist_t[idx_range]
        tau = robot.hist_tau[idx_range]
        Fe = robot.hist_Fe[idx_range]
        q_h = robot.hist_q[idx_range]
        
        pts = fk_3r_full(robot.q)
        link_line.set_data(pts[:,0], pts[:,1])
        ef_dot.set_data([pts[-1,0]], [pts[-1,1]])

        if robot.net.last_recv_time > 0 and (time.time() - robot.net.last_recv_time) < 1.0:
            conn_text.set_text('CONECTADO')
            conn_text.set_color('#69FF47')
        else:
            conn_text.set_text('SIN CONEXIÓN')
            conn_text.set_color('#FF4444')

        # Actualizar indicador de estado del esclavo (guía para operación a ciegas)
        slave_st = robot.net.slave_state
        state_text.set_text(slave_st)
        state_colors = {
            "APROXIMACIÓN": "#888888",
            "CONTACTO": "#FF6B6B",
            "INSERCIÓN": "#FFD700",
            "COMPLETADO ": "#69FF47",
        }
        state_text.set_color(state_colors.get(slave_st, "#888888"))
        
        t_win = 5.0
        mask = (t > robot.t - t_win) if robot.t > t_win else np.ones(n, bool)
        
        for i, ln in enumerate(lines_tau):
            ln.set_data(t[mask], tau[mask, i])
        ax_tau.set_xlim(max(0, robot.t - t_win), max(t_win, robot.t))
        ax_tau.relim(); ax_tau.autoscale_view(scalex=False)
        
        line_Fx.set_data(t[mask], Fe[mask, 0])
        line_Fy.set_data(t[mask], Fe[mask, 1])
        ax_force.set_xlim(max(0, robot.t - t_win), max(t_win, robot.t))
        ax_force.relim(); ax_force.autoscale_view(scalex=False)
        
        for i, ln in enumerate(lines_q):
            ln.set_data(t[mask], q_h[mask, i])
        ax_q.set_xlim(max(0, robot.t - t_win), max(t_win, robot.t))
        ax_q.relim(); ax_q.autoscale_view(scalex=False)
        
        return [link_line, ef_dot] + lines_tau + [line_Fx, line_Fy] + lines_q

    key_map = {'up': 'up', 'down': 'down', 'left': 'left', 'right': 'right',
               'w': 'up', 's': 'down', 'a': 'left', 'd': 'right'}

    def on_key_press(event):
        print(f"[KEY] press: '{event.key}'")  # debug
        mapped = key_map.get(event.key)
        if mapped:
            robot.keys_held.add(mapped)
        elif event.key == 'q': robot.gripper_open = True
        elif event.key == 'e': robot.gripper_open = False
        elif event.key == 'escape':
            running[0] = False
            plt.close()

    def on_key_release(event):
        mapped = key_map.get(event.key)
        if mapped:
            robot.keys_held.discard(mapped)

    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('key_release_event', on_key_release)
    
    ani = animation.FuncAnimation(fig, animate, interval=50, blit=False, cache_frame_data=False)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TE3001B Robot Maestro Peg-in-Hole")
    parser.add_argument("--slave-ip", default="127.0.0.1", help="IP del PC esclavo (default: loopback)")
    args = parser.parse_args()
    main(args.slave_ip)