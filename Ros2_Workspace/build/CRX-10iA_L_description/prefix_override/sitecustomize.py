import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/salman/Ros2_Workspace/install/CRX-10iA_L_description'
