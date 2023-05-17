///////////////////////////////////////
/// 640x480 version! 16-bit color
/// This code will segfault the original
/// DE1 computer
/// compile with
/// gcc multiple_boids_VGA.c -o mult_boids -O3 -lm
///
///////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h> 
#include <sys/shm.h> 
#include <sys/mman.h>
#include <sys/time.h> 
#include <math.h>
//#include "address_map_arm_brl4.h"

// video display
#define SDRAM_BASE            0xC0000000
#define SDRAM_END             0xC3FFFFFF
#define SDRAM_SPAN			  0x04000000
// characters
#define FPGA_CHAR_BASE        0xC9000000 
#define FPGA_CHAR_END         0xC9001FFF
#define FPGA_CHAR_SPAN        0x00002000
/* Cyclone V FPGA devices */
#define HW_REGS_BASE          0xff200000
//#define HW_REGS_SPAN        0x00200000 
#define HW_REGS_SPAN          0x00005000 

// graphics primitives
void VGA_text (int, int, char *);
void VGA_text_clear();
void VGA_box (int, int, int, int, short);
void VGA_rect (int, int, int, int, short);
void VGA_line(int, int, int, int, short) ;
void VGA_Vline(int, int, int, short) ;
void VGA_Hline(int, int, int, short) ;
void VGA_disc (int, int, int, short);
void VGA_circle (int, int, int, int);

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(((( signed long long)(a))<< 15) / (b)) 
#define sqrtfix(a) (float2fix15(sqrt(fix2float15(a))))

// Wall detection
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

// uS per frame
#define FRAME_RATE 1000000/30 //33000
#define PI 3.1415927
// #define BLACK 0x00
// #define WHITE 0xFF

// 16-bit primary colors
#define red  (0+(0<<5)+(31<<11))
#define dark_red (0+(0<<5)+(15<<11))
#define green (0+(63<<5)+(0<<11))
#define dark_green (0+(31<<5)+(0<<11))
#define blue (31+(0<<5)+(0<<11))
#define dark_blue (15+(0<<5)+(0<<11))
#define yellow (0+(63<<5)+(31<<11))
#define cyan (31+(63<<5)+(0<<11))
#define magenta (31+(0<<5)+(31<<11))
#define black (0x0000)
#define gray (15+(31<<5)+(51<<11))
#define white (0xffff)
int colors[] = {red, dark_red, green, dark_green, blue, dark_blue, 
		yellow, cyan, magenta, gray, black, white};

// pixel macro
#define VGA_PIXEL(x,y,color) do{\
	int  *pixel_ptr ;\
	pixel_ptr = (int*)((char *)vga_pixel_ptr + (((y)*640+(x))<<1)) ; \
	*(short *)pixel_ptr = (color);\
} while(0)

// the light weight buss base
void *h2p_lw_virtual_base;

// pixel buffer
volatile unsigned int * vga_pixel_ptr = NULL ;
void *vga_pixel_virtual_base;

// character buffer
volatile unsigned int * vga_char_ptr = NULL ;
void *vga_char_virtual_base;

// /dev/mem file id
int fd;

//? Define the turn factor for adjusting the boid's direction when near an edge
fix15 turnfactor = float2fix15(0.2);
//? Define the visual range for boids to perceive other boids within this distance
fix15 visualRange = float2fix15(75);
//? Define the protected range for boids to maintain a safe distance from other boids
fix15 protectedRange = float2fix15(8);
//? Define the centering factor for how strongly boids try to match the position of other boids
fix15 centeringfactor = float2fix15(0.0005);
//? Define the avoidance factor for how strongly boids try to avoid other boids within the protected range
fix15 avoidfactor = float2fix15(0.05);
//? Define the matching factor for how strongly boids try to match the velocity of other boids
fix15 matchingfactor = float2fix15(0.05);
//? Define the maximum speed a boid can move
fix15 maxspeed = float2fix15(6);
//? Define the minimum speed a boid can move
fix15 minspeed = float2fix15(3);
//? Define the maximum bias value to change boid's direction
fix15 maxbias = float2fix15(0.01);
//? Define the increment value for bias adjustment
fix15 bias_increment = float2fix15(0.00004);
//? Define the initial bias value for adjusting the boid's direction
fix15 biasval = float2fix15(0.001);


//*========================================================================
//* Boid Initialization and Spawning
//*========================================================================
typedef struct list {
   fix15 pos_x;
   fix15 pos_y;
   fix15 vx; 
   fix15 vy ;
   int color; 
} Boid;



//!========================================================================
//! NUMBER OF BOIDS
//!========================================================================
#define NUM_BOIDS 20
Boid boid_list[NUM_BOIDS];
//!========================================================================

void boid_initialize(Boid* b) {
    b->pos_x = 0;
    b->pos_y = 0;
    b->vx = 0;
    b->vy = 0;
    b->color = white;
}

// bound type 
fix15 old_bound_type = 0;
fix15 bound_type = 0;


//*========================================================================
//* Creating Multiple Boids
//*========================================================================
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // Start in frame with random location
  *x = int2fix15(rand() % (540-100) + 100);
  *y = int2fix15(rand() % (380-100) + 100);

  // random total speed
  float random_speed = (float)rand()/(float)(RAND_MAX) * (6-3) + 3;
  float random_angle = (float)rand()/(float)(RAND_MAX) * (2*PI);

  *vx = float2fix15(random_speed *  (float) cos ((double) random_angle));
  *vy = float2fix15(random_speed *  (float) sin ((double) random_angle));

}


//*========================================================================
//* Updating Boid velocities per interaction with boundaries
//*========================================================================
static inline void boundary_check(int boid_num) {
  fix15* x = &boid_list[boid_num].pos_x;
  fix15* y = &boid_list[boid_num].pos_y;
  fix15* vx = &boid_list[boid_num].vx;
  fix15* vy = &boid_list[boid_num].vy;

  // If the boid is near an edge, make it turn by turnfactorWW
  if (hitTop(*y)) {
      *vy = *vy + turnfactor ;
  }
  if (hitBottom(*y)) {
      *vy = *vy - turnfactor ;
  } 
  if (hitRight(*x)) {
      *vx = *vx - turnfactor;
  }
  if (hitLeft(*x)) {
      *vx = *vx + turnfactor;
  } 

}


//*========================================================================
//* Updating Each Boid
//*========================================================================
void boid_update(int cur)
{
    fix15* x = &boid_list[cur].pos_x;
    fix15* y = &boid_list[cur].pos_y;
    fix15* vx = &boid_list[cur].vx;
    fix15* vy = &boid_list[cur].vy;
    
    fix15 xpos_avg = 0, ypos_avg = 0, xvel_avg = 0, yvel_avg = 0,  close_dx = 0, close_dy = 0;
    fix15 neighboring_boids = 0;
    int i;
    for (i = 0; i < 50; i++) {
      if(i != cur) {
      fix15* x_o = &boid_list[i].pos_x;
      fix15* y_o = &boid_list[i].pos_y;
      fix15* vx_o = &boid_list[i].vx;
      fix15* vy_o = &boid_list[i].vy;
      // Compute differences in x and y coordinates
      fix15 dx = *x - *x_o;
    //   if(dx > int2fix15(320)) {dx = dx - int2fix15(640);}
    //   if(dx < int2fix15(-320)) {dx = dx + int2fix15(640);}
      fix15 dy = *y - *y_o;
    //   if(dy > int2fix15(240)) {dy = dy - int2fix15(480);}
    //   if(dy < int2fix15(-240)) {dy = dy + int2fix15(480);}

      // Are both those differences less than the visual range?
      if (dx < visualRange && dy < visualRange && dx > -visualRange && dy > -visualRange) {
          // If so, calculate the squared distance
          fix15 squared_distance = multfix15(dx,dx) + multfix15(dy,dy);

          // Is squared distance less than the protected range?
          if (squared_distance < multfix15(protectedRange,protectedRange)) { 

              // If so, calculate difference in x/y-coordinates to nearfield boid
              close_dx += dx;
              close_dy += dy;
          }

          // If not in protected range, is the boid in the visual range?
          else if (squared_distance < multfix15(visualRange,visualRange)) {

              // Add other boid's x/y-coord and x/y vel to accumulator variables
              xpos_avg += *x_o;
              ypos_avg += *y_o;
              xvel_avg += *vx_o;
              yvel_avg += *vy_o;

              // Increment number of boids within visual range
              neighboring_boids += int2fix15(1);
          }
        }
      }    
    }

    // If there were any boids in the visual range . . .            
    if (neighboring_boids > 0) {

        // Divide accumulator variables by number of boids in visual range
        // xpos_avg = divfix(xpos_avg,neighboring_boids);
        // ypos_avg = divfix(ypos_avg,neighboring_boids);
        // xvel_avg = divfix(xvel_avg,neighboring_boids);
        // yvel_avg = divfix(yvel_avg,neighboring_boids);

        xpos_avg = float2fix15( fix2float15(xpos_avg) / fix2float15(neighboring_boids) );
        ypos_avg = float2fix15( fix2float15(ypos_avg) / fix2float15(neighboring_boids) );
        xvel_avg = float2fix15( fix2float15(xvel_avg) / fix2float15(neighboring_boids) );
        yvel_avg = float2fix15( fix2float15(yvel_avg) / fix2float15(neighboring_boids) );
        
        // Add the centering/matching contributions to velocity
        *vx = *vx + multfix15((xpos_avg - *x),centeringfactor) +  multfix15((xvel_avg - *vx), matchingfactor);                      

        *vy = *vy + multfix15((ypos_avg - *y),centeringfactor) +  multfix15((yvel_avg - *vy), matchingfactor);
    } 

    //  printf("%d \n", fix2int15(*vx));
    // Add the avoidance contribution to velocity
    *vx = *vx + multfix15(close_dx,avoidfactor);
    *vy = *vy + multfix15(close_dy,avoidfactor);

    boundary_check(cur);


     
    // If the boid has a bias, bias it!
    // biased to right of screen
    if (cur >= 0 && cur < NUM_BOIDS/2) {
        *vx = multfix15((int2fix15(1) - biasval),(*vx)) + biasval;
    }
        
    // biased to left of screen
    else if (cur >= 25 && cur < NUM_BOIDS) {
        *vx = multfix15((int2fix15(1) - biasval),(*vx)) - biasval;
    }

       

    // Calculate the boid's speed
    // Slow step! Lookup the "alpha max plus beta min" algorithm
    fix15 temp_add = multfix15(*vx,*vx) + multfix15(*vy,*vy);
    fix15 speed = sqrtfix( temp_add );

    // printf("%d \n", fix2int15(speed));
    // // Enforce min and max speeds
    if (speed < minspeed && speed != 0) {
         *vx = multfix15(divfix(*vx,speed),minspeed);
         *vy = multfix15(divfix(*vy,speed),minspeed);
    }

    if (speed > maxspeed) {
         *vx = multfix15(divfix(*vx,speed),maxspeed);
         *vy = multfix15(divfix(*vy,speed),maxspeed);
    }


    // Update position using velocity
    *x = *x + *vx ;
    if(*x > int2fix15(640)) {
		*x = *x - int2fix15(15);
		*vx = -*vx;
	}
    if(*x < int2fix15(0)) {
		*x = *x + int2fix15(15);
		*vx = -*vx;
	}
	
    *y = *y + *vy ;
    if(
		*y > int2fix15(480)) {*y = *y - int2fix15(15);
		*vy = -*vy;
	}
    if(
		*y < int2fix15(0)) {*y = *y + int2fix15(15);
		*vy = -*vy;
	}
    
}



//*========================================================================
//* Arena Creation and Boundaries
//*========================================================================
//? Must be redrawn every time to ensure that Arena does not get overwritten
//? with black spots
void drawArena() {
    //plot a line 
    //at x1,y1 to x2,y2 with color 
    VGA_line(100,100,100,380,(short)white);
    VGA_line(540,100,540,380,(short)white);
    VGA_line(100,100,540,100,(short)white);
    VGA_line(100,380,540,380,(short)white);
}


//*========================================================================
//* Measure Time for EACH iteration of the MAIN function
//*========================================================================
struct timeval t1, t2;
double elapsedTime;

//!========================================================================
//! Main
//!========================================================================
int main(void)
{
  	
	// === need to mmap: =======================
	// FPGA_CHAR_BASE
	// FPGA_ONCHIP_BASE      
	// HW_REGS_BASE        
  
	// === get FPGA addresses ==================
    // Open /dev/mem
	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) 	{
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}
    
    // get virtual addr that maps to physical
	h2p_lw_virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );	
	if( h2p_lw_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap1() failed...\n" );
		close( fd );
		return(1);
	}
    

	// === get VGA char addr =====================
	// get virtual addr that maps to physical
	vga_char_virtual_base = mmap( NULL, FPGA_CHAR_SPAN, ( 	PROT_READ | PROT_WRITE ), MAP_SHARED, fd, FPGA_CHAR_BASE );	
	if( vga_char_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap2() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA LED control 
	vga_char_ptr =(unsigned int *)(vga_char_virtual_base);

	// === get VGA pixel addr ====================
	// get virtual addr that maps to physical
	vga_pixel_virtual_base = mmap( NULL, SDRAM_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SDRAM_BASE);	
	if( vga_pixel_virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap3() failed...\n" );
		close( fd );
		return(1);
	}
    
    // Get the address that maps to the FPGA pixel buffer
	vga_pixel_ptr =(unsigned int *)(vga_pixel_virtual_base);

	// ===========================================

	/* create a message to be displayed on the VGA 
          and LCD displays */
	char text_top_row[40] = "DE1-SoC ARM/FPGA\0";
	char text_bottom_row[40] = "Cornell ece5760\0";
	char text_next[40] = "Graphics primitives\0";
	char num_string[20], time_string[20] ;
	char color_index = 0 ;
	int color_counter = 0 ;
	
	// position of disk primitive
	int disc_x = 0;
	// position of circle primitive
	int circle_x = 0 ;
	// position of box primitive
	int box_x = 5 ;
	// position of vertical line primitive
	int Vline_x = 350;
	// position of horizontal line primitive
	int Hline_y = 250;

	//VGA_text (34, 1, text_top_row);
	//VGA_text (34, 2, text_bottom_row);
	// clear the screen
	VGA_box (0, 0, 639, 479, 0x0000);
	// clear the text
	VGA_text_clear();
	// write text
	VGA_text (10, 1, text_top_row);
	VGA_text (10, 2, text_bottom_row);
	VGA_text (10, 3, text_next);
	
	// R bits 11-15 mask 0xf800
	// G bits 5-10  mask 0x07e0
	// B bits 0-4   mask 0x001f
	// so color = B+(G<<5)+(R<<11);


    //? Spawning multiple boids
    int i;
    for(i = 0; i < NUM_BOIDS; i++) {
        boid_initialize(&boid_list[i]);
        // if (i >= 0 && i < 25) {
        //   boid_list[i].color = white;
        // }
        // else if (i >= 25 && i < 50) {
        //   boid_list[i].color = red;
        // }
        spawnBoid(&boid_list[i].pos_x, &boid_list[i].pos_y, &boid_list[i].vx, &boid_list[i].vy);
    }

	while(1) 
	{

		drawArena();

        //? start the timer
        gettimeofday(&t1, NULL);
        
        for (i = 0; i < NUM_BOIDS; i++) {
            int x = fix2int15(boid_list[i].pos_x);
            int y = fix2int15(boid_list[i].pos_y);
            
            //? Erase the previous boid position by drawing a black box
            VGA_box(x, y, x + 2, y + 2, black);
            
            

            //? Update boid's position, velocity, and behavior using the boid_update function
            boid_update(i);

            //? Get the updated position of the boid
            x = fix2int15(boid_list[i].pos_x);
            y = fix2int15(boid_list[i].pos_y);

            //? Draw the updated boid position using a different color (e.g., white)
            VGA_box(x, y, x + 2, y + 2, boid_list[i].color);
        }


        //? Stop timer
        gettimeofday(&t2, NULL);
        //? Calculate elapsed time in microseconds
        elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000.0;      // sec to us
        elapsedTime += (t2.tv_usec - t1.tv_usec);   // us 

        //? Convert elapsed time to a string for display
        sprintf(time_string, "T = %6.0f uSec  ", elapsedTime);
        
        //? Display elapsed time on the VGA screen at position (10, 4)
        VGA_text (10, 4, time_string);

        //? Set frame rate by waiting for a fixed time (17 ms) before next iteration
		usleep(10700);

		
	} // end while(1)
} // end main

/****************************************************************************************
 * Subroutine to send a string of text to the VGA monitor 
****************************************************************************************/
void VGA_text(int x, int y, char * text_ptr)
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset;
	/* assume that the text string fits on one line */
	offset = (y << 7) + x;
	while ( *(text_ptr) )
	{
		// write to the character buffer
		*(character_buffer + offset) = *(text_ptr);	
		++text_ptr;
		++offset;
	}
}

/****************************************************************************************
 * Subroutine to clear text to the VGA monitor 
****************************************************************************************/
void VGA_text_clear()
{
  	volatile char * character_buffer = (char *) vga_char_ptr ;	// VGA character buffer
	int offset, x, y;
	for (x=0; x<79; x++){
		for (y=0; y<59; y++){
	/* assume that the text string fits on one line */
			offset = (y << 7) + x;
			// write to the character buffer
			*(character_buffer + offset) = ' ';		
		}
	}
}

/****************************************************************************************
 * Draw a filled rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_box(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	for (row = y1; row <= y2; row++)
		for (col = x1; col <= x2; ++col)
		{
			//640x480
			//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
			// set pixel color
			//*(char *)pixel_ptr = pixel_color;	
			VGA_PIXEL(col,row,pixel_color);	
		}
}

/****************************************************************************************
 * Draw a outline rectangle on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_rect(int x1, int y1, int x2, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
	if (x1>x2) SWAP(x1,x2);
	if (y1>y2) SWAP(y1,y2);
	// left edge
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
		
	// right edge
	col = x2;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
	
	// top edge
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);
	}
	
	// bottom edge
	row = y2;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);
	}
}

/****************************************************************************************
 * Draw a horixontal line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Hline(int x1, int y1, int x2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (x1>x2) SWAP(x1,x2);
	// line
	row = y1;
	for (col = x1; col <= x2; ++col){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);		
	}
}

/****************************************************************************************
 * Draw a vertical line on the VGA monitor 
****************************************************************************************/
#define SWAP(X,Y) do{int temp=X; X=Y; Y=temp;}while(0) 

void VGA_Vline(int x1, int y1, int y2, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col;

	/* check and fix box coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (y2<0) y2 = 0;
	if (y1>y2) SWAP(y1,y2);
	// line
	col = x1;
	for (row = y1; row <= y2; row++){
		//640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10)    + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;	
		VGA_PIXEL(col,row,pixel_color);			
	}
}


/****************************************************************************************
 * Draw a filled circle on the VGA monitor 
****************************************************************************************/

void VGA_disc(int x, int y, int r, short pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++)
		for (xc = -r; xc <= r; xc++)
		{
			col = xc;
			row = yc;
			// add the r to make the edge smoother
			if(col*col+row*row <= rsqr+r){
				col += x; // add the center point
				row += y; // add the center point
				//check for valid 640x480
				if (col>639) col = 639;
				if (row>479) row = 479;
				if (col<0) col = 0;
				if (row<0) row = 0;
				//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
				// set pixel color
				//*(char *)pixel_ptr = pixel_color;
				VGA_PIXEL(col,row,pixel_color);	
			}
					
		}
}

/****************************************************************************************
 * Draw a  circle on the VGA monitor 
****************************************************************************************/

void VGA_circle(int x, int y, int r, int pixel_color)
{
	char  *pixel_ptr ; 
	int row, col, rsqr, xc, yc;
	int col1, row1;
	rsqr = r*r;
	
	for (yc = -r; yc <= r; yc++){
		//row = yc;
		col1 = (int)sqrt((float)(rsqr + r - yc*yc));
		// right edge
		col = col1 + x; // add the center point
		row = yc + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		col = -col1 + x; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
	for (xc = -r; xc <= r; xc++){
		//row = yc;
		row1 = (int)sqrt((float)(rsqr + r - xc*xc));
		// right edge
		col = xc + x; // add the center point
		row = row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
		// left edge
		row = -row1 + y; // add the center point
		//check for valid 640x480
		if (col>639) col = 639;
		if (row>479) row = 479;
		if (col<0) col = 0;
		if (row<0) row = 0;
		//pixel_ptr = (char *)vga_pixel_ptr + (row<<10) + col ;
		// set pixel color
		//*(char *)pixel_ptr = pixel_color;
		VGA_PIXEL(col,row,pixel_color);	
	}
}

// =============================================
// === Draw a line
// =============================================
//plot a line 
//at x1,y1 to x2,y2 with color 
//Code is from David Rodgers,
//"Procedural Elements of Computer Graphics",1985
void VGA_line(int x1, int y1, int x2, int y2, short c) {
	int e;
	signed int dx,dy,j, temp;
	signed int s1,s2, xchange;
     signed int x,y;
	char *pixel_ptr ;
	
	/* check and fix line coordinates to be valid */
	if (x1>639) x1 = 639;
	if (y1>479) y1 = 479;
	if (x2>639) x2 = 639;
	if (y2>479) y2 = 479;
	if (x1<0) x1 = 0;
	if (y1<0) y1 = 0;
	if (x2<0) x2 = 0;
	if (y2<0) y2 = 0;
        
	x = x1;
	y = y1;
	
	//take absolute value
	if (x2 < x1) {
		dx = x1 - x2;
		s1 = -1;
	}

	else if (x2 == x1) {
		dx = 0;
		s1 = 0;
	}

	else {
		dx = x2 - x1;
		s1 = 1;
	}

	if (y2 < y1) {
		dy = y1 - y2;
		s2 = -1;
	}

	else if (y2 == y1) {
		dy = 0;
		s2 = 0;
	}

	else {
		dy = y2 - y1;
		s2 = 1;
	}

	xchange = 0;   

	if (dy>dx) {
		temp = dx;
		dx = dy;
		dy = temp;
		xchange = 1;
	} 

	e = ((int)dy<<1) - dx;  
	 
	for (j=0; j<=dx; j++) {
		//video_pt(x,y,c); //640x480
		//pixel_ptr = (char *)vga_pixel_ptr + (y<<10)+ x; 
		// set pixel color
		//*(char *)pixel_ptr = c;
		VGA_PIXEL(x,y,c);			
		 
		if (e>=0) {
			if (xchange==1) x = x + s1;
			else y = y + s2;
			e = e - ((int)dx<<1);
		}

		if (xchange==1) y = y + s2;
		else x = x + s1;

		e = e + ((int)dy<<1);
	}
}