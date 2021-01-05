#!/usr/bin/env bash

### This script creates a public and private key pair for the present user, if one doesn't exist yet
### And then adds it to authorized_keys

## Check if I have openssh-server:
ISOPENSSHSERVERINSTALLED=`dpkg -l | grep "openssh-server"`

if [ -z "${ISOPENSSHSERVERINSTALLED}" ]
then
  printf "$0: ERROR: "
  printf "Openssh-server not installed.\n"
  printf "\n"
  printf "Install it with:\n\t\$ sudo apt-get install openssh-server\n"
  printf "\n"
  printf "If you want a non standard configuration, you should do the key exchange manually.\nMaybe check this out:\nhttps://serverfault.com/questions/252300/copying-local-ssh-key-to-remote-host-if-it-doesnt-exist-already/252317\n"
else
  ## this creates keys in ~/.ssh/ if they are not there already. or it should. if they are it should do nothing.
  cat /dev/zero | ssh-keygen -q -N ""

  ## Creates authorized_keys if it isn't there already
  touch ~/.ssh/authorized_keys

  ## I am unsure if I need this, but these are the right permissions anyway
  chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys

  ## I need to authorize myself

  KEY=$(cat ~/.ssh/id_rsa.pub)
  if [ -z \"\$(grep \"$KEY\" ~/.ssh/authorized_keys )\" ]
  then
    echo $KEY >> ~/.ssh/authorized_keys
    echo key added.
  fi

  ## Now you need to be using the authorized_keys list, otherwise this will not work.

  sudo cp /etc/ssh/sshd_config /etc/ssh/sshd_config.backup --backup
  ## This may fail with double quotes instead of single and the $0 instead of
  ## keys.sh, specially if this file is renamed to have like and underscore or
  ## something else that sed doesn't like
  ## old line
  #sed -i 's/[\s#]*AuthorizedKeysFile\s*(\s*.ssh\/authorized_keys )*/## Modified by dop_tch\/keys.sh ##\nAuthorizedKeysFile .ssh\/authorized_keys /' -r /etc/ssh/sshd_config
  SEDEXPRESSION="s/[\s#]*AuthorizedKeysFile\s*(\s*.ssh\/authorized_keys )*/## Modified by dop_tch\/$0 ##\nAuthorizedKeysFile .ssh\/authorized_keys /"
  sed -i $SEDEXPRESSION -r /etc/ssh/sshd_config
  sudo service ssh restart
fi
