ARG ROS_VERSION=foxy

FROM ros2_ws:${ROS_VERSION}

# install node
SHELL ["/bin/bash", "--login", "-i", "-c"]
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN source $HOME/.bashrc && nvm install --lts
RUN nvm use --lts

WORKDIR ${HOME}
COPY --chown=${USER} ./frontend ./frontend
COPY --chown=${USER} ./backend ./backend
COPY --chown=${USER} ./module_library ./module_library

WORKDIR ${HOME}/frontend
RUN npm install

WORKDIR ${HOME}/backend
# RUN pip3 install -r requirements.txt
SHELL ["/bin/bash", "--login", "-c"]

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]