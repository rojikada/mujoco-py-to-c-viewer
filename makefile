COMMON=-O2 -I../include -L../bin -std=c++11 -stdlib=libc++ -mavx -pthread

all:
	clang++ $(COMMON) viewer.cpp      -lmujoco200 -lglfw.3 -o ../bin/viewer