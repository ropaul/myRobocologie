#! /bin/bash

#PBS -N nomexperience
#PBS -o tmp/test_job.out
#PBS -b tmp/test_job.err
#PBS -M tonadresse@bla.fr
#PBS -l walltime=23:59:59
#PBS -l nodes=1:ppn=16
#PBS -d /home/.../
./roborobo config/...
