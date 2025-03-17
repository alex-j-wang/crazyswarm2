import argparse
import os
import subprocess

import docker
import docker.errors
import docker.types

ROS_SOURCE_COMMAND = "source /opt/ros/humble/setup.bash"


def lookForDevContainer(docker_client, dev_image_label="crazyswarm2:latest"):
    """
    look for the dev image locally; build if it doesn't exist
    """
    try:
        dev_container = docker_client.images.get(dev_image_label)
    except docker.errors.ImageNotFound:
        print("Image not found, building")
        curr_dir = os.path.abspath(os.getcwd())
        dev_container, build_log = docker_client.images.build(
            path=curr_dir, dockerfile=os.path.join(curr_dir, "Dockerfile"), tag=dev_image_label, rm=True
        )
    finally:
        return dev_container


def startDevContainer(docker_client, image):
    """
    mount current repo and sub folders containing code into the container
    """
    ros_ws_path = "/ros_ws/src"
    current_dir = os.path.abspath(os.getcwd())
    all_targets = os.listdir(current_dir)
    volumes = {}
    for target in all_targets:
        volumes[os.path.join(current_dir, target)] = {"bind": os.path.join(ros_ws_path, target), "mode": "rw"}

    # Create the container
    container = docker_client.containers.create(
        image,
        name="anoop-crazyswarm2",
        volumes=volumes,
        command="bash",
        tty=True,
        stdin_open=True,
        auto_remove=True,
    )
    container.start()
    return container


def buildSourceFiles(running_container):
    worker_cnt = os.cpu_count() - 2
    build_cmd = (
        "source /opt/ros/humble/setup.bash && cd /ros_ws && colcon build --symlink-install --parallel-workers "
        + str(worker_cnt)
    )
    
    print("Building with " + str(worker_cnt) + " workers")
    try:
        _, stream = running_container.exec_run(f"bash -c '{build_cmd}'", stream=True, tty=True, workdir="/ros_ws")
        for data in stream:
            print(data.decode("utf-8"), end="")
    except KeyboardInterrupt:
        print("Aborting build, stopping container...")
        running_container.stop()
        raise KeyboardInterrupt
    else:
        print("Build complete")


def main():
    parser = argparse.ArgumentParser(prog="develop.py", description="Launch crazyswarm2 dev container")
    parser.add_argument("-b", "--build", help="Build the container", action="store_true")
    parser.add_argument("-r", "--run", help="Run the container after build", action="store_true")
    args = parser.parse_args()
    client = docker.from_env()
    image = lookForDevContainer(client)
    container = startDevContainer(client, image)
    if args.build:
        try:
            buildSourceFiles(container)
        except KeyboardInterrupt:
            container.stop()
            exit(1)

    if not args.build or args.run:
        launch_cmd = ROS_SOURCE_COMMAND + " && bash"
        print("Starting shell")
        subprocess.run(["docker", "exec", "-it", container.id, "bash", "-c", launch_cmd])
        container.stop()


if __name__ == "__main__":
    main()