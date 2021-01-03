# rosdop
A Ros package to give ros docker build, mount and run abilities

Alright, first: this package is probably useless. You should use docker compose and docker swarm for what it does, or just plain docker.

Also, this is a wrapper which tries to put the parameters on a ros master so ros can try to control it. I haven't figured out all the angles, but I think the structure that makes sense is like this:


## The docker image building thing: TUB

I am calling it a tub because it needs a name and tub is short and sounds concrete. Stuff gets really abstract really fast, so this is a shortcut that works for me. This should be a master repo.

This thing has the dockerfile with all the scripts you need to create it and directory called the workspace. The workspace is a shared folder between the docker host and the docker guest(?). This is the tubvolume. This workspace will contain stuff that is shared that you want to make persistent and all the catkin worspace. The catkin_ws should be submoduled into the tub repo.

# TO-DOs:


## Tub rosdop interface

This is still not implemented, but I suppose it should be a ros package that will depend on rosdop stuff

## Building the image

Here I will try to do it 2 different ways that maybe make sense. One is just using the catkin_make and add a target which will build the image again. But this maybe will get slow, so I will perhaps put some option to disable this behaviour and a node to call the build command. Sounds a bit convoluted, dunno
