# PRM - Programação de Robôs Móveis

**Disciplina SSC0712** 
Oferecida para os cursos de Engenharia de Computação e áreas afins na **USP São Carlos**

Este repositório contém o material da disciplina *Programação de Robôs Móveis*, focada no desenvolvimento de soluções em robótica móvel utilizando **ROS 2 Humble** e o simulador **Gazebo Fortress**.

## 📦 Tecnologias utilizadas

- ROS 2 Humble
- Gazebo Fortress
- Python
- RViz / Gazebo GUI
- [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard)

---

## 🚀 Como utilizar o pacote

### 1. Instalar dependências

Instale as dependências do pacote com:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> Certifique-se de ter rodado previamente `sudo rosdep init` e `rosdep update`, se for a primeira vez usando o `rosdep`.

### 2. Compilar o workspace

Certifique-se de estar na **raiz do seu workspace** (geralmente `~/ros2_ws`) antes de compilar:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select prm
```

### 3. Atualizar o ambiente do terminal

```bash
source install/local_setup.bash
```

---

## 🧪 Executando a simulação

### Terminal 1 - Iniciar o mundo no Gazebo

```bash
source install/local_setup.bash
ros2 launch prm inicia_simulacao.launch.py
```

### Terminal 2 - Carregar o robô no ambiente

Em um **novo terminal** (não se esqueça de `source install/local_setup.bash`):

```bash
source install/local_setup.bash
ros2 launch prm carrega_robo.launch.py
```

### Terminal 3 – Ative os controladores ROS 2 Control

Em outro terminal:

```bash
source install/local_setup.bash
ros2 control load_controller --set-state active joint_state_broadcaster
ros2 control load_controller --set-state active diff_drive_base_controller

```

### Terminal 4 - Inicie a ponte da câmera (image bridge)


```bash
source install/local_setup.bash
ros2 launch meu_prm bridge_camera.launch.py
```

### Terminal 5 – Inicie a missão do robô (navegação)
```bash
source install/local_setup.bash
ros2 run meu_prm controle_missao
```

### Alternativa: controle manual (teclado)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
#### Instalar `teleop_twist_keyboard` (caso não esteja disponível)

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```
### Sobre o projeto
Este pacote implementa um sistema de controle e navegação automática para um robô móvel simulado, com sensores (IMU, LIDAR e câmera RGB) no ambiente do Gazebo. O comportamento principal envolve exploração do ambiente, seguimento de paredes, detecção de uma bandeira e planejamento de rota até ela usando o algoritmo A* (A-estrela).
