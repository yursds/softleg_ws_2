#!/bin/bash

# Trova l'ID del primo container in esecuzione
CONTAINER_ID=$(docker ps -q)

# Se non ci sono container in esecuzione, esci con un messaggio
if [ -z "$CONTAINER_ID" ]; then
    echo "Nessun container in esecuzione."
    exit 1
fi

# Apri una sessione interattiva nel container
docker exec -it "$CONTAINER_ID" /bin/bash

