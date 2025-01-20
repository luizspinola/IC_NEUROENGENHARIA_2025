import os
import socket
import sys

DEFAULT_NETWORK_PREFIX = '172.30.1' # Endereço IPv4 adaptador ethernet
SVR_DATA_MC_IP = '230.0.0.0'
SVR_CFG_MC_IP = '230.0.0.1'
SEN_CFG_MC_IP = '230.0.0.2'
PORT = 50000

def time():
    if os.name == 'nt':
        import timeit
        return timeit.default_timer()
    else:
        import time
        return time.time()

def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    try:
        # doesn't even have to be reachable
        s.connect(('10.254.254.254', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip
    
    
def get_default_ip():
    itf = '0.0.0.0' 
    itfs = socket.gethostbyname_ex(socket.gethostname())[2] # Gethostname retorna o nome do dispositivo a partir do qual 
                                                            # o Python está rodando. Gethostbyname_ex retorna uma tupla
                                                            # contendo o nome do host, outros names para o host e o IP
    for ip in itfs:
        if ip.startswith(DEFAULT_NETWORK_PREFIX):
            itf = ip
            break
    return itf

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    try:
        # PyInstaller creates a temp folder and stores path in _MEIPASS
        base_path = sys._MEIPASS
    except Exception:
        base_path = os.path.abspath(".")

    return os.path.join(base_path, relative_path)