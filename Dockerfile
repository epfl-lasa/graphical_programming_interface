 ARG ROS_VERSION=foxy

FROM ros2_ws:${ROS_VERSION}

# Install node
SHELL ["/bin/bash", "--login", "-i", "-c"]
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN source $HOME/.bashrc && nvm install --lts
RUN nvm use --lts

# Install supervisor to be able to run backend and frontend at the same time
USER root
RUN apt-get update && apt-get install -y supervisor
RUN mkdir -p /var/log/supervisor
COPY supervisord.conf /etc/supervisor/conf.d/supervisord.conf

WORKDIR ${HOME}
# COPY --chown=${USER} ./frontend ./frontend
COPY --chown=${USER} ./src ./src
# COPY --chown=${USER} ./module_library ./module_library
# COPY --chown=${USER} ./run.py .

# WORKDIR ${HOME}/frontend
# RUN npm install

WORKDIR ${HOME}/src/backend
RUN pip3 install -r requirements.txt
SHELL ["/bin/bash", "--login", "-c"]



# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
