#!/bin/bash
# Salva il contenuto del repository in git
#unset DISPLAY
unset SSH_ASKPASS

echo "git add ."
git add .
echo "git commit -a -m \"$1\""
git commit -a -m "$1"
echo "git push"
git push origin master

exit 0
