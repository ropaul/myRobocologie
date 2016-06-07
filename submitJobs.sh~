#!/bin/bash

jobName=$1
nb_runs=$2;

if [ ! $# == 2 ]; then
  echo "  Usage: $0 jobname.script nb_of_replications"
  exit
fi

i=0
while [ $i -lt $nb_runs ]
do
	qsub $1
	sleep 2
	i=$[$i+1]
done
