# Docker Notes
- 1 container has 1 process only while 1 VM can have multiple processes

## Creating and starting your first container

1. Create container

       docker create -it --name=my_container_name ubuntu bash

1. Start container

       docker start my_container_name

1. Attach to container

       docker attach my_container_name

1. Detach from container (escape sequence)
   using `<Ctrl-d>` or `exit` will kill the container

       <Ctrl-p><Ctrl-q>

1. Stop container

       docker stop my_container_name

## Creating your own image

1. Create repo in [Docker Hub](https://cloud.docker.com/repository/list)

1. Tag an existing image

       docker tag <existing_image> <Docker Hub username>/<repo_name>:<tag_name>
       docker tag <existing_image> <Docker Hub username>/<repo_name>:latest

1. Push to repo

       docker push <Docker Hub username>/<repo_name>

1. Pull repo

       docker pull <Docker Hub username>/<repo_name>:<tag_name>

   If `<tag_name>` is empty, defaults to `latest`

## Dockerfile

Automated script for setting up a Docker image

Standard name of Dockerfile is `Dockerfile`

[Difference between `CMD` and `RUN`](https://stackoverflow.com/questions/37461868/difference-between-run-and-cmd-in-a-docker-file)

1. Edit `Dockerfile`

   ```
   FROM <image_name>:<tag_name>
   MAINTAINER <Name> <<email>>
   RUN <setup_command1>
   RUN <setup_command2>
   # Optional: Document which port will be used
   # Note that actual port specification should be passed through
   # docker run -p <local_port>:<container_port>
   EXPOSE <container_port>
   COPY <local_file_relative_to_Dockerfile> <container_file>
   CMD ["<runtime_cmd>", "<input>"]
   ```

   Example:

   ```
   FROM ubuntu:latest
   MAINTAINER Rufus Wong <rcywongaa@gmail.com>
   RUN apt update && apt install -y openssh-server
   RUN mkdir -p /var/run/sshd
   # Assuming there exists file directory/script.sh
   COPY directory /directory/
   RUN chmod 777 /directory/script.sh
   CMD ["/directory/script.sh"]
   CMD ["echo","hello world"]
   ```

1. Build Docker image

    docker build -t <image_name>:<tag_name> .

1. Run Docker image

       docker run --name=my_container_name <image_name>:<tag_name>

   `CMD` is overrideable by the `docker run` command

       docker run --name=my_container_name <image_name>:<tag_name> echo "override CMD"

### Give user sudo permission
```
FROM ubuntu:14.04
RUN useradd --create-home --shell /bin/bash username
RUN echo "username ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
USER username
WORKDIR /home/username
```

## Docker Networking

By default, all docker containers are connected together via a shared virtual network

    docker network ls

- `bridge`: Default name of the virtual network
- `host`: ???
- `null`: ??? (seldom used)

    docker network inspect <virtual_network_name> 

- Gateway is the IP of the host on the virtual network
  - Containers will use the host as a gateway to connect outside

### Port Forwarding

Ports used by containers by default are only accessible through `<container_ip>:<container_port>` 

To expose it to be accessible through `<host_ip>:<host_port>`

    docker run -p <host_port>:<container_port>

### Creating a new private network

    docker network create <virtual_network_name>

### Move container to new network

    docker network disconnect <old_network_name> <container_name>
    docker network connect --ip <desired_ip> <new_network_name> <container_name>

## Useful tools to install

    apt install \
    iputils-ping \
    iproute2

## Commands

### Docker Run (= docker create + docker start + docker attach)

    docker run -it --rm --name=<container_name> <command>

- `-i` to create interactive session
- `-t` to use tty
- `--rm` to automatically remove on exit

### List currently running docker instances

    docker ps
    docker container ls

### List all currently available containers

    docker container ls -a

### List currently available images

    docker image ls

### Search for available images
<https://hub.docker.com/search/?type=image>

    docker search <image name>

### Remove container

    docker rm my_container_name # Only works if container stopped
    docker rm -f my_container_name # Force remove container even if running

### Pull image

    docker pull <image_name>

### Remove image

    docker rmi <image_name or image ID>

### Execute command in container and view results without attaching

    docker exec <container_name> <command>

Note that variable expansion occurs outside container, to make it occur inside:

    docker exec container bash -c 'echo "$ENV_VAR"'

### Extra resources / examples
<https://github.com/15Dkatz/docker-guides>
