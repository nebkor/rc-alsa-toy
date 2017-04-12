ALL := alsa-capture ne-alsa-capture glprog

all: $(ALL)
	@echo Done

alsa-capture: alsa-capture.c
	gcc -o $@ $< -lasound

ne-alsa-capture: ne_alsa_capture.c
	gcc -o $@ $< -lm -lrt -lasound -lrfftw -lfftw

glprog: ne_glprog.c
	gcc -o $@ $< -lglut -lGLU -lrt -lGL

clean:
	$(RM) $(ALL)

