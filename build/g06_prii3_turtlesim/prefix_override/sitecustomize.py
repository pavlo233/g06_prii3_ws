import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pavlo/g06_prii3_ws/install/g06_prii3_turtlesim'
