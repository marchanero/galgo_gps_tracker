#!/bin/bash

# Obtén todas las ramas remotas
git fetch --all

# Actualiza todas las ramas locales con las remotas
for branch in $(git branch -r | grep -v '\->' | grep -v 'HEAD' | sed 's/origin\///'); do
    git checkout $branch
    git pull origin $branch
done

# Vuelve a la rama original
git checkout -