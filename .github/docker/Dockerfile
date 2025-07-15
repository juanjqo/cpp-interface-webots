FROM juanjqo/ubuntu_24_dqrobotics

SHELL ["/bin/bash", "-c"]

# Install Webots
RUN /bin/bash -c "mkdir -p /etc/apt/keyrings; cd /etc/apt/keyrings; wget -q https://cyberbotics.com/Cyberbotics.asc" 
RUN /bin/bash -c "echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | sudo tee /etc/apt/sources.list.d/Cyberbotics.list"


