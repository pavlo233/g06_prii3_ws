import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/disa/proyect3/pruebitaspr2/mov6/install/mov6'
