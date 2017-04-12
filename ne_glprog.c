/*
 * prog : ne_glprog.c
 *
 * desc : demo opengl (glut) program for displaying a realtime
 *        audio frequency spectrum using values obtained from 
 *        posix shm. The values are the processed output of an
 *        fft performed on the audio stream by a separate program
 *        module. For example usage, check out:
 *
 *       	http://nairobi-embedded.org/alsa_daq_and_rt_fft.html
 *        http://nairobi-embedded.org/alsa_ladspa_demo_dsp_prototyping.html
 *
 * notes:
 *        This code is a ridiculous hack! I only posses casual
 *        skills in OpenGL.
 *
 * compile with:
 *
 * 	"gcc -Wall -O2 ne_glprog.c -o ne_glprog -lglut -lGLU -lrt"
 *
 * Siro Mugabi, nairobi-embedded.org
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>

#include "ne_common.h"
static struct ne_glprog_fband_data *fband_data_map;

/* glut window width and height */
#define WINWIDTH 570
#define WINHEIGHT 320
static int win_x = WINWIDTH;
static int win_y = WINHEIGHT;
static int win_id;

/* ====== Text Rendering Routines ====== */
void renderBitmapString(float x, float y, void *font,char *string)
{
  char *c;
  glRasterPos2f(x, y);
  for (c=string; *c != '\0'; c++) {
    glutBitmapCharacter(font, *c);
  }
}

void setOrthographicProjection() {

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0, win_x, 0, win_y);
	glScalef(1, -1, 1);
	glTranslatef(0, -win_y, 0);
	glMatrixMode(GL_MODELVIEW);
}

void resetPerspectiveProjection() {
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

/* ======= Frequency-band bars rendering routine ====== */
#define BARWIDTH 30
#define BARSPACING 7
#define X_BAROFFSET (BARSPACING + BARWIDTH)
#define Y_BAROFFSET 30
void draw_bands(void) {
	int i;

	glColor3f(.4f, .4f, .4f);
	for(i = 0; i < NE_GLPROG_FBANDS; i++)
		glRectf(BARSPACING + (i * X_BAROFFSET), Y_BAROFFSET, 
						X_BAROFFSET + (i * X_BAROFFSET), 
						fband_data_map[i].fband_magn + Y_BAROFFSET);
}

/* ======= Display engine ======= */
static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity (); 
	gluOrtho2D ( 0.0, win_x, 0.0, win_y ); 
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display ( void )
{
	glutSwapBuffers ();
}

static int time, timebase = 0, frame = 0;
#define SLEN 64
static char s1[SLEN], s2[SLEN];
#define FBSLEN 16
static char fbands[NE_GLPROG_FBANDS][FBSLEN];
static void display_func ( void )
{
	int i;

	pre_display ();
	draw_bands();
	frame++;
	time=glutGet(GLUT_ELAPSED_TIME);
	if (time - timebase > 1000) {
		snprintf(s1, SLEN, "FPS:%4.2f",
			frame*1000.0/(time-timebase));
		timebase = time;		
		frame = 0;
	}

	glColor3f(0.0f,1.0f,1.0f);
	glPushMatrix();
	glLoadIdentity();
	setOrthographicProjection();
	renderBitmapString(30, 10, GLUT_BITMAP_8_BY_13,s1);
	renderBitmapString(30, 25, GLUT_BITMAP_8_BY_13, 
										(char *) "Esc or 'q' to Quit");
	for(i = 0; i < NE_GLPROG_FBANDS; i++){
		snprintf(s2, SLEN, "%s", fbands[i]);
		if(i % 2)
			renderBitmapString(X_BAROFFSET * i, win_y-20, 
					               GLUT_BITMAP_8_BY_13, s2);
		else
			renderBitmapString(X_BAROFFSET * i, win_y-5, 
					               GLUT_BITMAP_8_BY_13, s2);
	}
	glPopMatrix();
	resetPerspectiveProjection();
	post_display ();
}

static void idle_func ( void )
{
	glutSetWindow ( win_id );
	/* display them freq bars */
	glutPostRedisplay ();
}

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 27 : /* ESC */
		case 'q':
		case 'Q':
			exit (EXIT_SUCCESS);
			break;			
	}
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

/* ======== Initialization Routines ======= */

static void misc_init(void)
{
	int i;
	float val;

	for(i = 0; i < NE_GLPROG_FBANDS; i++){
		val = ne_glprog_fband[i]/1000.0f;
		if((int)val == 0)
			snprintf(fbands[i], FBSLEN, "%dHz", (int)(val * 1000));
		else
			snprintf(fbands[i], FBSLEN, "%.1fKHz", val);
	}
}

static void *shm_init(const char *const shm_filename)
{
	int fd, ret = -1;
	struct stat stat;
	size_t shm_filesize;
	void *map = NULL;

	fd = shm_open(shm_filename, O_RDWR , (mode_t) 0666);
	if(fd < 0){
     prerr("Error opening \"%s\", %s\n", 
				 shm_filename, strerror(errno));
     return NULL;
	}

	ret = fstat(fd, &stat);
	if(ret < 0){
		prerr("%s\n", strerror(errno));
		goto exit;
	}
	
	shm_filesize = stat.st_size;
  map = mmap(0, shm_filesize, PROT_READ | PROT_WRITE, 
			       MAP_SHARED, fd, 0);
	if(map	== MAP_FAILED) {
  	prerr("%s. Is \"%s\" of zero-length?\n", 
				   strerror(errno), shm_filename);
		map = NULL;
		goto exit;
	}

exit:
	close(fd);
	return map;
}
 
static void open_glut_window ( void )
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "NE | RT Audio Freq Spectrum" );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	pre_display ();
	glutKeyboardFunc ( key_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}

int main(int argc, char **argv) 
{
	glutInit(&argc, argv);
	fband_data_map = (struct ne_glprog_fband_data *)
		shm_init(NE_GLPROG_FBAND_DATA_FILE);
	if(!fband_data_map)
		exit(EXIT_FAILURE);
	misc_init();
	open_glut_window();
	glutMainLoop();
	exit(EXIT_SUCCESS);
}

