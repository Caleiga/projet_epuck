# 	Fichier repris du cours de l'EPFL "syst�mes embarqu�s et robotique" (MICRO-335)
#	Modifi� par Julian B�r et F�lix Laurent
#	Derni�re modification: 16.05.2021

#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = project

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../../lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
	./audio_processing.c \
	./communications.c \
	./fft.c \
	./image_processing.c \
	./drive.c \

#Header folders to include
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile