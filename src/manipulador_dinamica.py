import roboticstoolbox as rtb
import numpy as np
from math import cos, sin, atan2, acos, sqrt, pi
import matplotlib.pyplot as plt
import time

class RRP:
    def __init__(self):
        l1 = 0.4 #400mm
        l2 = 0.5*l1
        l3 = 1.25*l1

        self.a1 = l1; self.a2 = l2; self.a3 = 0
        self.alpha1 = 0; self.alpha2 = np.pi; self.alpha3 = 0
        self.d1 = 0; self.d2 = 0; self.d3 = 0
        self.theta1 = 0; self.theta2 = 0; self.theta3 = 0

        w = l1*0.25
        h = l1*0.15

        w3 = l1/20
        h3 = l2/20

        #considerando densidade do alúminio
        m1 = 2700*l1*w*h
        m2 = 2700*l2*w*h
        m3 = 2700*l3*w*h

        #Aproximando para paralelepípedos
        Ixx1 = (1/12)*m1*(pow(h,2)+pow(w,2))
        Iyy1 = (1/12)*m1*(pow(h,2)+pow(l1,2))
        Izz1 = (1/12)*m1*(pow(l1,2)+pow(w,2))

        Ixx2 = (1/12)*m2*(pow(h,2)+pow(w,2))
        Iyy2 = (1/12)*m2*(pow(h,2)+pow(l2,2))
        Izz2 = (1/12)*m2*(pow(l2,2)+pow(w,2))

        Ixx3 = (1/12)*m2*(pow(l3,2)+pow(w3,2))
        Iyy3 = (1/12)*m2*(pow(l3,2)+pow(h3,2))
        Izz3 = (1/12)*m2*(pow(h3,2)+pow(w3,2))

        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(d=self.d1, a=self.a1, alpha=self.alpha1, offset=self.theta1, m=m1, r=[l1/2,0,0], I=np.diag([Ixx1, Iyy1, Izz1])),
            rtb.RevoluteDH(d=self.d2, a=self.a2, alpha=self.alpha2, offset=self.theta2, m=m2, r=[l2/2,0,0], I=np.diag([Ixx2, Iyy2, Izz2])),
            rtb.PrismaticDH(theta=self.theta3, a=self.a3, alpha=self.alpha3, offset=self.d3, m=m3, r=[0,0,l3/2], I=np.diag([Ixx3, Iyy3, Izz3]))
        ])

        # para visualizar a declaração do manipulador
        print(self.robot)

    def cinematica_direta(self, q):

        theta1, theta2, d3 = q

        T01 = np.array([
            [cos(theta1), -sin(theta1)*cos(self.alpha1), sin(theta1)*sin(self.alpha1), self.a1*cos(theta1)],
            [sin(theta1), cos(theta1)*cos(self.alpha1), -cos(theta1)*sin(self.alpha1), self.a1*sin(theta1)],
            [0, sin(self.alpha1), cos(self.alpha1), self.d1],
            [0, 0, 0, 1]
        ])

        T12 = np.array([
            [cos(theta2), -sin(theta2)*cos(self.alpha2), sin(theta2)*sin(self.alpha2), self.a2*cos(theta2)],
            [sin(theta2), cos(theta2)*cos(self.alpha2), -cos(theta2)*sin(self.alpha2), self.a2*sin(theta2)],
            [0, sin(self.alpha2), cos(self.alpha2), self.d2],
            [0, 0, 0, 1]
        ])

        T23 = np.array([
            [cos(self.theta3), -sin(self.theta3)*cos(self.alpha3), sin(self.theta3)*sin(self.alpha3), self.a3*cos(self.theta3)],
            [sin(self.theta3), cos(self.theta3)*cos(self.alpha3), -cos(self.theta3)*sin(self.alpha3), self.a3*sin(self.theta3)],
            [0, sin(self.alpha3), cos(self.alpha3), d3],
            [0, 0, 0, 1]
        ])

        T03 = T01 @ T12 @ T23

        return T03

    def cinematica_inversa(self, p):

        x, y, z = p

        # junta 3 (prismática)
        q3 = -z

        # junta 2
        c2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2*self.a1*self.a2)
        c2 = np.clip(c2, -1, 1)

        s2 = sqrt(1 - c2**2)
        theta2 = atan2(s2, c2)   # solução cotovelo para cima
        # theta2 = atan2(-s2, c2)  # alternativa cotovelo para baixo

        # junta 1
        k1 = self.a1 + self.a2*cos(theta2)
        k2 = self.a2*sin(theta2)

        theta1 = atan2(y, x) - atan2(k2, k1)

        return np.array([theta1, theta2, q3])


    #Calculo da trajetória no espaço das juntas (Para a simulação da cinemática direta e inversa)
    def trajetoria_simulacao(self,q_i, q_f):
        # Posicoes inicial e final
        q_start = q_i
        q_end = q_f

        # Tempo de simulacao e dt
        time_s = 5.0
        dt = 0.05
        steps = int(time_s / dt)

        # Vetor de tempo
        t = np.linspace(0, time_s, steps)

        # Trajetória polinomial de 5º grau
        traj = rtb.jtraj(q_start, q_end, t)

        # Calcular torques usando dinâmica inversa
        tau = self._calcular_torques(traj.q, traj.qd, traj.qdd)

        # Plotar gráficos
        self._plotar_graficos(t, traj.q, traj.qd, traj.qdd, tau)

        # Animar trajetória
        self.robot.plot(traj.q, backend="pyplot", dt=dt, block=False)

        return 0
    
    #Calculo da trajetória no espaço cartesiano (Para a simulação da trajetória de escovação)
    def trajetoria_cartesiana(self, p_start, p_end, time_segment, dt):
        steps = int(time_segment / dt)
        t = np.linspace(0, time_segment, steps)

        tau = t / time_segment
        s = 10*tau**3 - 15*tau**4 + 6*tau**5

        p_traj = np.zeros((steps, 3))
        for i in range(3):
            p_traj[:, i] = p_start[i] + s * (p_end[i] - p_start[i])

        q = np.zeros((steps, 3))
        for k in range(steps):
            q[k, :] = self.cinematica_inversa(p_traj[k, :])

        qd = np.gradient(q, dt, axis=0, edge_order=2)
        qdd = np.gradient(qd, dt, axis=0, edge_order=2)

        self.q_all.append(q)
        self.qd_all.append(qd)
        self.qdd_all.append(qdd)
        self.t_all.append(t + self.t_acc)
        self.t_acc += t[-1]


    def trajetoria_escovacao(self):
        print("\n" + "="*60)
        print("INICIANDO TRAJETÓRIA")
        print("="*60)

        # Definir pontos da trajetória (x, y, z)
        p_home = np.array([0.5, 0.1, 0.0])
        p_above = np.array([0.4, 0.4, 0.0])
        p_brush1 = np.array([0.4, 0.4, -0.2])
        p_brush2 = np.array([0.4, -0.4, -0.2])


        # Parâmetros de tempo
        dt = 0.05
        time_segment = 2

        # Armazenar trajetória completa
        self.q_all = []
        self.qd_all = []
        self.qdd_all = []
        self.t_all = []
        self.t_acc = 0

        # Montando a trajetória
        self.trajetoria_cartesiana(p_home, p_above, time_segment, dt)
        self.trajetoria_cartesiana(p_above, p_brush1, time_segment, dt)
        self.trajetoria_cartesiana(p_brush1, p_brush2, 3, dt)
        self.trajetoria_cartesiana(p_brush2, p_brush1, 3, dt)
        self.trajetoria_cartesiana(p_brush1, p_brush2, 3, dt)
        self.trajetoria_cartesiana(p_brush2, p_brush1, 3, dt)
        self.trajetoria_cartesiana(p_brush1, p_above, time_segment, dt)
        self.trajetoria_cartesiana(p_above, p_home, time_segment, dt)        

        # Concatenar todos os segmentos
        q_total = np.vstack(self.q_all)
        qd_total = np.vstack(self.qd_all)
        qdd_total = np.vstack(self.qdd_all)
        t_total = np.concatenate(self.t_all)

        # Calcular torques usando dinâmica inversa
        tau = self._calcular_torques(q_total, qd_total, qdd_total)

        # Plotar gráficos
        self._plotar_graficos(t_total, q_total, qd_total, qdd_total, tau)

        # Animar trajetória
        print("\nAnimando trajetória...")
        self.robot.plot(q_total, backend="pyplot", dt=dt, block=False)

        return q_total, qd_total, qdd_total, t_total

    def _calcular_torques(self, q, qd, qdd):
        n_points = q.shape[0]
        tau = np.zeros((n_points, 3))

        for i in range(n_points):
            tau[i, :] = self.robot.rne(q[i, :], qd[i, :], qdd[i, :])
        return tau

    def _plotar_graficos(self, t, q, qd, qdd, tau):
        fig, axes = plt.subplots(4, 1, figsize=(12, 12))

        joint_names = ['Junta 1 (θ₁)', 'Junta 2 (θ₂)', 'Junta 3 (d₃)']
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

        # Gráfico de Posição
        for i in range(3):
            if i < 2:
                axes[0].plot(t, q[:, i], color=colors[i], linewidth=2, label=joint_names[i])
            else:
                ax0_twin = axes[0].twinx()
                ax0_twin.plot(t, q[:, i], color=colors[i], linewidth=2, label=joint_names[i], linestyle='--')
                ax0_twin.set_ylabel('Posição Junta 3 (m)', fontsize=8, color=colors[i])
                ax0_twin.tick_params(axis='y', labelcolor=colors[i])
                ax0_twin.legend(loc='upper right')

        axes[0].set_ylabel('Posição Juntas 1 e 2 (rad)', fontsize=8)
        axes[0].set_title('Posição das Juntas', fontsize=12, fontweight='bold')
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc='upper left')

        # Gráfico de Velocidade
        for i in range(3):
            if i < 2:
                axes[1].plot(t, qd[:, i], color=colors[i], linewidth=2, label=joint_names[i])
            else:
                ax1_twin = axes[1].twinx()
                ax1_twin.plot(t, qd[:, i], color=colors[i], linewidth=2, label=joint_names[i], linestyle='--')
                ax1_twin.set_ylabel('Velocidade Junta 3 (m/s)', fontsize=8, color=colors[i])
                ax1_twin.tick_params(axis='y', labelcolor=colors[i])
                ax1_twin.legend(loc='upper right')

        axes[1].set_ylabel('Velocidade Juntas 1 e 2 (rad/s)', fontsize=8)
        axes[1].set_title('Velocidade das Juntas', fontsize=12, fontweight='bold')
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(loc='upper left')

        # Gráfico de Aceleração
        for i in range(3):
            if i < 2:
                axes[2].plot(t, qdd[:, i], color=colors[i], linewidth=2, label=joint_names[i])
            else:
                ax2_twin = axes[2].twinx()
                ax2_twin.plot(t, qdd[:, i], color=colors[i], linewidth=2, label=joint_names[i], linestyle='--')
                ax2_twin.set_ylabel('Aceleração Junta 3 (m/s²)', fontsize=8, color=colors[i])
                ax2_twin.tick_params(axis='y', labelcolor=colors[i])
                ax2_twin.legend(loc='upper right')

        axes[2].set_ylabel('Aceleração Juntas 1 e 2 (rad/s²)', fontsize=8)
        axes[2].set_title('Aceleração das Juntas', fontsize=12, fontweight='bold')
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(loc='upper left')

        # Gráfico de Torque/Força
        for i in range(3):
            if i < 2:
                axes[3].plot(t, tau[:, i], color=colors[i], linewidth=2, label=joint_names[i])
            else:
                ax3_twin = axes[3].twinx()
                ax3_twin.plot(t, tau[:, i], color=colors[i], linewidth=2, label=joint_names[i], linestyle='--')
                ax3_twin.set_ylabel('Força Junta 3 (N)', fontsize=8, color=colors[i])
                ax3_twin.tick_params(axis='y', labelcolor=colors[i])
                ax3_twin.legend(loc='upper right')

        axes[3].set_xlabel('Tempo (s)', fontsize=8)
        axes[3].set_ylabel('Torque Juntas 1 e 2 (N·m)', fontsize=8)
        axes[3].set_title('Torque/Força das Juntas', fontsize=12, fontweight='bold')
        axes[3].grid(True, alpha=0.3)
        axes[3].legend(loc='upper left')
        plt.tight_layout()
        plt.show()



Solver = RRP()
qi = np.array([0, 0, 0])
while True:
    print("Escolha qual função deseja executar:")
    escolha = input("cd (Cinemática direta) / ci (Cinemática inversa) / te (Trajetória de escovação) / limpar (voltar a posição [0 0 0]) / s (Sair) \n")

    if escolha == "cd":
        valores = input("Insira os valores das 3 juntas sepadados por espaço (rotacionais em graus):")
        q = [float(x) for x in valores.split()]
        for i in range(2):
            q[i] = q[i]*np.pi/180
        Matriz = Solver.cinematica_direta(q)
        print(Matriz)
        Solver.trajetoria_simulacao(qi,q)
        print(f"Posição final: {Matriz[:3, 3]}")
        qi = q

    elif escolha == "ci":
        valores = input("Insira os valores as coordenadas x y z separadas por espaço:")
        p = [float(x) for x in valores.split()]
        q = Solver.cinematica_inversa(p)
        Solver.trajetoria_simulacao(qi,q)
        q_graus = [q[0]*180/np.pi, q[1]*180/np.pi, q[2]]
        print(f"Posição das juntas: theta1={q_graus[0]:.2f}°, theta2={q_graus[1]:.2f}°, d3={q[2]:.2f}")
        for i in range(2):
            q[i] = q_graus[i]*np.pi/180
        Matriz = Solver.cinematica_direta(q)
        print(f"Posição final: {Matriz[:3, 3]}")
        qi = q

    elif escolha == "te":
        q_traj, qd_traj, qdd_traj, t_traj = Solver.trajetoria_escovacao()
        qi = q_traj[-1]

    elif escolha == "s":
        break

    elif escolha == "limpar":
        qi = np.array([0, 0, 0])
        Solver.trajetoria_simulacao(qi,qi)

    else:
        print("ERRO!")
        print("Digite cd, ci, pp ou s")
