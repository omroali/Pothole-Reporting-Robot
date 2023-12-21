# Build Stage
FROM python:3.10
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

## TODO: make sure that ROS2 is built properly

## TODO: make sure that LIMO stuff is built properly?

COPY submodules/ submodules/
COPY ros_workspace/ ros_workspace/
COPY code/ code/

# fetching submodules
RUN cd submodules/CMP9767_LIMO && \
    git checkout main && \
    git pull origin main

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["entrypoint.sh"]



# TODO: source and build the ros_workspace
# TODO: setup the limo robot
