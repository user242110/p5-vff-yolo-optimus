import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alumnos/angelfr/Descargas/ROBOTICA/PRACTICAS/p5-vff-yolo-optimus/p5_vff_yolo/install/p5_vff_yolo'
