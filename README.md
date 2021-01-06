# rosdop
A Ros package to give ros docker build, mount and run abilities

Alright, first: this package is probably useless. You should use docker compose and docker swarm for what it does, or just plain docker.

Also, this is a wrapper which tries to put the parameters on a ros master so ros can try to control it. I haven't figured out all the angles, but I think the structure that makes sense is like this:

## The docker image building package thing: TUB

ROS package + dockerfile == TUB

I am calling it a tub because it needs a name and tub is short and sounds concrete. Stuff gets really abstract really fast, so this is a shortcut that works for me. This should be a master repo with submodules.

This thing has the dockerfile with all the scripts you need to create it and directory called the workspace. The workspace is a shared folder between the docker host and the container. This share is done via sshfs and is called the tubvolume. This workspace will contain stuff that is shared that you want to make persistent and all the catkin worspace.

I recommend that the catkin_ws that runs inside the container to be submoduled into the tub repo.

I find it useful to have notebooks so you can use jupyter. So maybe another directory in the share is ./notebooks.

## Other shares

For some of the stuff I was doing it was convenient to have a shared folder for datasets. This I share between containers, the same for all; this is not the case for the TubVolume, that each tub should have one.

I don't think this is the final solution here, but I am using it like this and it seems to make sense. I've made this a samba share and mount it to every ros docker container. An NFS share would be better because sometimes samba is weird and it messes up file properties.

Pandas has problems reading a csv inside a samba share (maybe I can change some settings when I mount it to make this go away) and git rev-parse HEAD was failing as well (and consequently weights and biases which uses it for version control). So here I probably need to either find the right parameters for mounting the samba share or change this for something else. 

# TO-DOs:

- anonymize everything and make sure I can run multiple tubs.
- make templates(?) so that tubs are easier to make
