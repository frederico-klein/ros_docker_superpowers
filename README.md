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


# running

we need to clear all the docker containers (everything with docker ps that is ros related must be dead and removed)

we also need routing to inside the containers and the shared samba we sometimes use to save things (TODO: although a samba share is a bad idea because it does not save file permissions properly. better would be an NFS share)

we automated this with some scripts in catkin_newidea

they are numbered in order

on lavine:
    ./00.sh
    ./05.sh
    ./10.sh # will launch core + ros docker master + volume

on another terminal we need also rosmasq (TODO: maybe it would make sense to group those together..)

    ./20.sh

IMPORTANT: we cannot pile up machines that are not registered on the dns. This is a mistake and we need to rewrite things so that this is not possible. Now it breaks everything, so rosmasq needs to be the second thing to run!

on poop:

    /00.sh
    /10.sh

## we are not even close to being finished.

As you may recall, we did not manage to do this without a computer serving as router. Surely if every container was privileged, I suppose we could have done this, but currently the solution was to use a whole computer just for this. this computer is named debian.

now we need to restart the dnsmasq running on the debian router (we have 2 levels of dns and this points to the original router 192.168.0.1 and the docker router, so that every ros computer is set by default to use the debian router. I think this is a horrible design, but I couldn't come up with anything better and simpler. TODO: simplify this, probably with a normal user in the ros_dockers and running all of them as privileged with proper keys, so things are relatively safe ).

for sanity check, check the routes in debian:

    ssh debian
    su
    route

this should show routes to both lavine and poop. if it doesn't, it means the routing service did not work, so you need to figure out why.

check that you can get the docker containers ips from names as they are running already:

    dig torch_machine4.poop
    dig torch_machine4.lavine

if it doesn't work then check the log in debian:

    tail /tmp/dnsmasq.log

sometimes it gets too long and we have to delete it.

    rm /tmp/dnsmasq.log

restart the server:

    service dnsmasq restart

they should resolve just fine. now check that you can ping them

    ping torch_machine4.poop
    ping torch_machine4.lavine

### testing connectivity

if netcat isnt working then anythint else will (from http://wiki.ros.org/ROS/NetworkSetup ). you need to install it on the containers

    apt install netcat -y

then run it

    nc -l -p 1234

on another computer than go:

   nc torch_machine4.poop 1234

they should copy stuff. if it doesn't then something is wrong. if it cant get the name it is the dns. maybe the routes are off, you have to figure it out.

I've set docker to publish_all; there is another option called EXPOSE in the dockerfile which I think does the same, but I am not sure how they interact. with publish all i can netcat to any port, so I believe that is sufficient.

### getting ready to run remote rosnodes

I should have automated this, but there is so much to do.

We need the correct keys with correct cyphers everywhere. This is painful, but you need to run:

    ssh -oHostKeyAlgorithms='ssh-rsa' root@torch_machine4.lavine
    ssh -oHostKeyAlgorithms='ssh-rsa' root@torch_machine4.poop

on every docker host and in every container,

##actually running the nodes:

now everything should be fine to start the nodes:

   roslaunch ros_example_torch_classifier sm.launch
