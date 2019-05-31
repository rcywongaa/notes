#!/bin/bash

cat ~/.ssh/id_rsa.pub | ssh -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no "${1}" "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >>  ~/.ssh/authorized_keys"
