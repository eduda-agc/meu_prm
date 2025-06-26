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
cd ~/prm_ws
colcon build --packages-select prm
source install/setup.bash
ros2 launch prm inicia_simulacao.launch.py
```

### Terminal 2 - Carregar o robô no ambiente

Em um **novo terminal** (não se esqueça de `source install/local_setup.bash`):

```bash
source install/setup.bash
ros2 launch prm carrega_robo.launch.py
```

### Terminal 3 – Inicie a missão!

Em outro terminal:

```bash
source install/setup.bash
ros2 run prm controle_missao
```
### Alternativa: controle manual (teclado)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
#### Instalar `teleop_twist_keyboard` (caso não esteja disponível)

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```
### Ponto de vista do robô
```bash
ros2 run rqt_gui rqt_gui
```

### Sobre o projeto
Este pacote implementa um sistema de controle e navegação automática para um robô móvel simulado, com sensores (IMU, LIDAR e câmera RGB) no ambiente do Gazebo. O comportamento principal envolve exploração do ambiente, seguimento de paredes, desvio de obstáculos e detecção da bandeira pela cor.

________________________________________________________________________________________________________________________________________________________________________________

### Trabalho 2
No trabalho 2 há uma Atividade Extensionista. Minha proposta final foi fazer um tutorial básido de Ros2 para iniciantes, contendo conceito e testes com turtlesim sobre:
- Instalação de Ros2
- Executaveis
- Topicos
- Serviços
- Parametros
- Ações
- Workspace
- Colcon Biuld
- Create Packages
- Publisher e Subscribers
- Launch Files
Todos em uma página Notion, para ficar mais fácil a cópia e cola, além da custimização. Segue o link: [Tutorial Ros2 Básico](https://www.notion.so/21ecbc4955798044a1bcc2af3a0dcbe4?source=copy_link)

Segue também o relatório da Atividade Extensionista: [Relatório](https://docs.google.com/document/d/1FXz7KUlFhLeUCHKfRZ7RURuUNr3TeapHrsSWpx5PjzQ/edit?usp=sharing)
