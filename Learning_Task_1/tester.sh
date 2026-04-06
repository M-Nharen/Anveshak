#!/bin/bash

correct=0
total=0

for i in ./arrow/*.jpg;do
    if [[ -n $(echo $i | egrep "left") ]]; then
        output=$(python3 Learning_Assignment_1.py $i | egrep LEFT)
        if [[ -n $output ]]; then
            (( correct+=1 ))
        fi
    fi
    if [[ -n $(echo $i | egrep "right") ]]; then
        output=$(python3 Learning_Assignment_1.py $i | egrep RIGHT)
        if [[ -n $output ]]; then
            (( correct+=1 ))
        fi
    fi
    (( total+=1 ))
done


echo "Correct images: $correct/$total"