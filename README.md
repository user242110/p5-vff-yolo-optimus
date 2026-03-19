[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/7dvYcpAO)
# p5_vff_yolo_follow_person

En esta práctica haremos que el robot busque y siga a una persona.

Para ello crearemos una máquina de estados con 2 estados: 
- Buscando: El robot girará sobre si mismo para buscar una persona. Si se detecta una persona, se pasa al estado Siguiendo.
- Siguiendo: El robot seguirá a la persona más grande que se detecte. Para ello se publicará un vector atractivo hacia esa persona que el robot seguirá. Si no se detecta a nadie durante 1 segundo, se pasa al estado Buscando.

Utiliza las instrucciones de la wiki de [asr-clase](https://github.com/URJC-teaching/asr-clase/tree/py) para lanzar yolo, y los nodos vff_control/vff_controller_node, vff_control/yolo_class_detector_node_2d.py y camera/yolo_detection_node.

Escribe un launch para lanzar todos los nodos necesarios con el comando ros2 launch.
