import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pavlo/g06_prii3_ws/src/install/g01_prii3_turtlesim'
