#!/bin/sh
for i in {0..10}
do
for j in {0..10}
do
        ./placer $i $j
        sleep 5
done
done
