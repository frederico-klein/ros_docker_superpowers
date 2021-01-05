docker container rm $(docker container ls -aq)
docker volume rm $(docker volume ls -qf dangling=true)
