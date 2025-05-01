#!/bin/bash

# Name of semaphore and shared memory
SEM_NAME="/valor_data_mutex"
SHM_NAME="/valor_pointcloud_data"

echo "Cleaning up interprocess resources..."

# Remove semaphore if it exists
if [ -e /dev/shm/sem.$SEM_NAME ] || [ -e /dev/shm$SEM_NAME ]; then
    echo "Removing semaphore: $SEM_NAME"
    rm -f /dev/shm/sem.$SEM_NAME 2>/dev/null
    rm -f /dev/shm$SEM_NAME 2>/dev/null
else
    echo "Semaphore not found: $SEM_NAME"
fi

# Remove shared memory if it exists
if [ -e /dev/shm$SHM_NAME ]; then
    echo "Removing shared memory: $SHM_NAME"
    rm -f /dev/shm$SHM_NAME
else
    echo "Shared memory not found: $SHM_NAME"
fi

echo "Cleanup complete!"
