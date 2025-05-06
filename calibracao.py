import numpy as np
import socket
import time
import struct
from dataprocessing import MadgwickFilter  # Importando o filtro do seu módulo

# Configurações de rede (ajuste conforme necessário)
SENSOR_IP = '192.168.1.100'  # IP base dos sensores
BASE_PORT = 50000

class SensorReader:
    def __init__(self, sensor_id):
        self.sensor_id = sensor_id
        self.port = BASE_PORT + sensor_id
        self.sock = None
        self.connected = False
        self.filter = MadgwickFilter()  # Filtro para cálculo do quatérnio
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z

    def connect(self):
        """Conecta ao sensor"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2)
        try:
            self.sock.connect((SENSOR_IP, self.port))
            self.connected = True
            print(f"Conectado ao sensor {self.sensor_id} em {SENSOR_IP}:{self.port}")
            return True
        except Exception as e:
            print(f"Erro ao conectar ao sensor {self.sensor_id}: {str(e)}")
            return False

    def read_data(self):
        """Lê e processa os dados do sensor"""
        if not self.connected:
            return False

        try:
            data = self.sock.recv(1024)
            if not data:
                return False

            # Processar dados brutos (ajuste conforme seu protocolo)
            if len(data) >= 29:  # Tamanho para CMD_DATA_AGM
                # Extrair dados (exemplo - ajuste para seu formato real)
                ax, ay, az = struct.unpack_from('>hhh', data, 13)
                gx, gy, gz = struct.unpack_from('>hhh', data, 19)
                mx, my, mz = struct.unpack_from('>hhh', data, 25)

                # Converter para unidades físicas (ajuste conforme necessário)
                accel = np.array([ax, ay, az]) * 0.001
                gyro = np.array([gx, gy, gz]) * 0.001
                mag = np.array([mx, my, mz]) * 0.001

                # Atualizar filtro e obter quatérnio
                self.filter.update(gyro, accel, mag)
                self.quaternion = self.filter.quaternion

                return True

        except socket.timeout:
            return True
        except Exception as e:
            print(f"Erro ao ler do sensor {self.sensor_id}: {str(e)}")
            return False

    def close(self):
        """Fecha a conexão"""
        if self.sock:
            self.sock.close()
        self.connected = False

def quaternion_multiply(q1, q2):
    """Multiplica dois quatérnios q1 * q2"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quaternion_conjugate(q):
    """Retorna o conjugado do quatérnio"""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def calculate_relative_quaternion(q_ref, q_target):
    """Calcula o quatérnio relativo entre dois sensores"""
    q_ref_conj = quaternion_conjugate(q_ref)
    q_relative = quaternion_multiply(q_target, q_ref_conj)
    # Normalizar
    norm = np.linalg.norm(q_relative)
    if norm > 0:
        q_relative /= norm
    return q_relative

def print_quaternion(label, q):
    """Exibe informações sobre um quatérnio"""
    angle = 2 * np.arccos(q[0]) * (180/np.pi)
    axis_norm = np.linalg.norm(q[1:])
    if axis_norm > 0:
        axis = q[1:] / axis_norm
    else:
        axis = np.array([0.0, 0.0, 0.0])
    
    print(f"\n{label}:")
    print(f"Quatérnio: w={q[0]:.4f}, x={q[1]:.4f}, y={q[2]:.4f}, z={q[3]:.4f}")
    print(f"Ângulo: {angle:.2f}°")
    print(f"Eixo: [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]")

def main():
    print("Iniciando calibração entre sensor 3 e 4...")
    
    # Criar leitores para os sensores
    sensor3 = SensorReader(3)
    sensor4 = SensorReader(4)
    
    # Conectar aos sensores
    if not sensor3.connect() or not sensor4.connect():
        print("Não foi possível conectar aos sensores")
        return
    
    try:
        print("\nAguardando dados dos sensores...")
        print("Mantenha os sensores na mesma orientação física para calibração")
        
        # Coletar amostras iniciais
        samples = []
        sample_count = 0
        max_samples = 50  # Número de amostras para calcular a média
        
        while sample_count < max_samples:
            # Ler dados de ambos os sensores
            sensor3.read_data()
            sensor4.read_data()
            
            # Coletar amostras periodicamente
            if sample_count % 5 == 0:  # A cada 5 iterações
                samples.append((sensor3.quaternion.copy(), sensor4.quaternion.copy()))
                print(".", end="", flush=True)
            
            sample_count += 1
            time.sleep(0.02)  # Pequena pausa
            
        print("\nCalculando quatérnio relativo...")
        
        # Calcular quatérnios médios
        q3_avg = np.mean([s[0] for s in samples], axis=0)
        q4_avg = np.mean([s[1] for s in samples], axis=0)
        
        # Normalizar quatérnios médios
        q3_avg /= np.linalg.norm(q3_avg)
        q4_avg /= np.linalg.norm(q4_avg)
        
        # Calcular quatérnio relativo
        q_relative = calculate_relative_quaternion(q3_avg, q4_avg)
        
        # Exibir resultados
        print("\n=== Resultados da Calibração ===")
        print_quaternion(f"Sensor 3 (Referência)", q3_avg)
        print_quaternion(f"Sensor 4 (Alvo)", q4_avg)
        print_quaternion("Quatérnio Relativo (4 em relação ao 3)", q_relative)
        
        print("\nUse este quatérnio para corrigir a orientação do Sensor 4:")
        print(f"[{q_relative[0]:.6f}, {q_relative[1]:.6f}, {q_relative[2]:.6f}, {q_relative[3]:.6f}]")
        
    except KeyboardInterrupt:
        print("\nCalibração interrompida pelo usuário")
    finally:
        sensor3.close()
        sensor4.close()

if __name__ == "__main__":
    main()