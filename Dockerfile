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
RUN cd submodules && \
  git checkout 417e961 # specifying hash to build on



# TODO: source and build the ros_workspace
# TODO: setup the limo robot
