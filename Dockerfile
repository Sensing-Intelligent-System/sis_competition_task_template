From argnctu/sis_base_image:v5


WORKDIR /root/sis_competition_task_template/
RUN rm -rf ./catkin_ws/src/competition_modules

RUN apt-get update && apt-get install -y ros-kinetic-move-base-msgs

COPY competition_modules/ ./catkin_ws/src
COPY master_task.launch ./catkin_ws/src/sis_arm/sis_arm_planning/launch/

COPY run_task.sh .
RUN /bin/bash -c "cd ~/sis_competition_task_template/ && source /opt/ros/kinetic/setup.bash && catkin_make -C catkin_ws/"
RUN /bin/bash -c "source ~/sis_competition_task_template/catkin_ws/devel/setup.bash"

CMD [ "./run_task.sh" ]
