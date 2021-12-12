#!/bin/sh
GOMP_CPU_AFFINITY="0 14" ./placer
sleep 5
GOMP_CPU_AFFINITY="7 5" ./placer
sleep 5
GOMP_CPU_AFFINITY="2 5" ./placer
sleep 5
GOMP_CPU_AFFINITY="17 15" ./placer
sleep 5
GOMP_CPU_AFFINITY="10 6" ./placer
sleep 5
GOMP_CPU_AFFINITY="13 3" ./placer
sleep 5
