docker container rm $(docker container ls -aq) 2>/dev/null
docker volume rm $(docker volume ls -qf dangling=true) 2>/dev/null
