# 🤖 ROS 2 - Práticas Fundamentais em Ambiente Docker 🐳

Repositório destinado às atividades práticas de comunicação essenciais em **robótica** e **sistemas distribuídos** utilizando **ROS 2 Humble**, explorando os padrões Publisher/Subscriber e Server/Client.

A atividade foi feita em um ambiente **containerizado via Docker**, garantindo reprodutibilidade e isolamento do sistema hospedeiro.

---

## 📚 Conceitos Abordados

O projeto é dividido em dois pacotes principais, cada um demonstrando um padrão de comunicação fundamental do ROS 2:

1. **Pacote `chatter` (Publisher/Subscriber)** 🛰️
   Um sistema de comunicação assíncrono onde um nó `talker` publica mensagens continuamente em um tópico, e um nó `listener` se inscreve neste tópico para recebê-las.

2. **Pacote `calculator_py` (Server/Client)** ⚙️
   Um sistema de comunicação síncrona onde um nó `calculator_server` oferece um serviço para somar dois números, e um nó `calculator_client` envia uma requisição com os números e aguarda o resultado.

---

## 🛠️ Pré-requisitos

* Docker instalado na máquina hospedeira.
* (Opcional, para Linux) Permissão para executar o Docker como usuário não-root.

---

## ⚙️ Configuração do Ambiente

Siga os passos abaixo para construir a imagem Docker e iniciar o container com o ambiente ROS 2.

**1. Clone o Repositório:**

```bash
git clone <https://github.com/GuilhermeAsura/edrom-ros2-core-practice.git>
cd <edrom-ros2-core-practice>
```

**2. Build da Imagem Docker:**
O `Dockerfile` na raiz do projeto contém todas as dependências necessárias.

```bash
docker build -t edrom_image .
```

**3. Inicie o Container:**
Um script de conveniência `start_container.sh` é fornecido para mapear os volumes necessários e iniciar o container.

```bash
# Se necessário, dê permissão de execução ao script
chmod +x start_container.sh

# Inicie o container
./start_container.sh
```

Após isso, você estará dentro do terminal do container, pronto para compilar e executar o código.

---

## 🏗️ Compilando o Workspace

Dentro do container, navegue até o workspace e compile os pacotes com `colcon`:

```bash
# O workspace já estará em /my_workspace/edrom_ws
cd /my_workspace/edrom_ws

# Compile todos os pacotes
colcon build
```

---

## ▶️ Executando as Atividades

Para cada atividade, você precisará de **dois terminais** abertos no container. Para abrir um segundo terminal, use o seguinte comando em uma nova aba do seu terminal hospedeiro:

```bash
docker exec -it edrom_ros2_container bash
```

**Lembre-se de carregar o ambiente em cada novo terminal com o comando:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
```

### 🚀 Atividade 1: Chatter (Publisher/Subscriber)

**Terminal 1 - Inicie o Listener:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run chatter listener
```

**Terminal 2 - Inicie o Talker:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run chatter talker
```

*O Terminal 1 começará a exibir as mensagens enviadas pelo Terminal 2.*

---

### 🔢 Atividade 2: Calculadora (Server/Client)

**Terminal 1 - Inicie o Servidor:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run calculator_py calculator_server
```

**Terminal 2 - Execute o Cliente:**
O cliente aceita dois números como argumentos de linha de comando.

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run calculator_py calculator_client 5 10
```

*O Terminal 2 exibirá o resultado (`15`), e o Terminal 1 mostrará um log de que a requisição foi recebida e processada.*

---

## 🙌 Agradecimentos

Este trabalho foi desenvolvido com o apoio e colaboração da **Equipe de Desenvolvimento em Robótica Móvel (EDROM)** da **Universidade Federal de Uberlândia (UFU)**. 🤖🎓

----

# 🤖 ROS 2 - Core Practices in Docker Environment 🐳

Repository dedicated to practical activities of **robotics** and **distributed systems** communication using **ROS 2 Humble**, exploring the Publisher/Subscriber and Server/Client patterns.

The activity was implemented in a **containerized Docker environment**, ensuring reproducibility and isolation from the host system.

---

## 📚 Concepts Covered

The project is divided into two main packages, each demonstrating a fundamental ROS 2 communication pattern:

1. **`chatter` Package (Publisher/Subscriber)** 🛰️
   An asynchronous communication system where a `talker` node continuously publishes messages to a topic, and a `listener` node subscribes to that topic to receive them.

2. **`calculator_py` Package (Server/Client)** ⚙️
   A synchronous communication system where a `calculator_server` node offers a service to sum two numbers, and a `calculator_client` node sends a request with the numbers and waits for the result.

---

## 🛠️ Requirements

* Docker installed on the host machine.
* (Optional, for Linux) Permission to run Docker without root privileges.

---

## ⚙️ Environment Setup

Follow the steps below to build the Docker image and start the container with the ROS 2 environment.

**1. Clone the Repository:**

```bash
git clone <https://github.com/GuilhermeAsura/edrom-ros2-core-practice.git>
cd <edrom-ros2-core-practice>
```

**2. Build the Docker Image:**
The `Dockerfile` in the root directory contains all required dependencies.

```bash
docker build -t edrom_image .
```

**3. Start the Container:**
A convenience script `start_container.sh` is provided to map the required volumes and start the container.

```bash
# If needed, give execution permission
chmod +x start_container.sh

# Start the container
./start_container.sh
```

After this, you will be inside the container terminal, ready to compile and run the code.

---

## 🏗️ Building the Workspace

Inside the container, navigate to the workspace and build the packages with `colcon`:

```bash
# The workspace will already be at /my_workspace/edrom_ws
cd /my_workspace/edrom_ws

# Build all packages
colcon build
```

---

## ▶️ Running the Activities

For each activity, you will need **two terminals** open inside the container. To open a second one, run the following command in a new tab of your host terminal:

```bash
docker exec -it edrom_ros2_container bash
```

**Remember to source the environment in each new terminal:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
```

### 🚀 Activity 1: Chatter (Publisher/Subscriber)

**Terminal 1 - Start the Listener:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run chatter listener
```

**Terminal 2 - Start the Talker:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run chatter talker
```

*Terminal 1 will begin displaying the messages sent from Terminal 2.*

---

### 🔢 Activity 2: Calculator (Server/Client)

**Terminal 1 - Start the Server:**

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run calculator_py calculator_server
```

**Terminal 2 - Run the Client:**
The client accepts two numbers as command-line arguments.

```bash
source /my_workspace/edrom_ws/install/setup.bash
ros2 run calculator_py calculator_client 5 10
```

*Terminal 2 will display the result (`15`), and Terminal 1 will log that the request was received and processed.*

---

## 🙌 Acknowledgments

This work was developed with the support and collaboration of the **Mobile Robotics Development Team (EDROM)** at the **Federal University of Uberlândia (UFU)**. 🤖🎓
