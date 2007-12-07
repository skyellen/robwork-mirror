//CREATE_ARTAG_MARKER.C - make patterns for ARTag - started Thur June 17/2004

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "artag_rev1.h"


void artag_load_char_data(void);
void artag_big_print_char(unsigned char*,int,int,int,int,int,int,int);
int artag_big_print_string(unsigned char*,int,int,int,int,char*,int,int);

unsigned char *mgcd;    //stored character pattern


int main(int argc, char *argv[])
{
int i,j,width,height,xoffset,yoffset;
unsigned char *image,*pattern_image;
char pgm_out_filename[256],comment[256];
char message[1024];
char label_on;
int scale,pattern_width;
int artag_id,invert;
FILE *out;



if((argc<2)||(argc>4))
   {printf("CREATE_ARTAG_MARKER.C creates a .PGM file from an ARTAG id#\n"
           "-needs 2 or 3 arguments\n"
           "-proper format is: CREATE_ARTAG_MARKER 157\n"
           "               or: CREATE_ARTAG_MARKER 157 image.pgm\n"
           "               or: CREATE_ARTAG_MARKER 157 image.pgm toplabel\n"
	   "-where 157 is an example ID (0-1023) \n"
	   "-if no output image filename (2nd argument) is given, the image will be written to\n"
	   " the default filename <artag_create_marker.pgm>\n"
	   "-3rd argument causes image to have label above with id\n"
	   );
    exit(1);}


sscanf(argv[1],"%d",&artag_id);
if(argc>=3) strcpy(pgm_out_filename,argv[2]);
else        strcpy(pgm_out_filename,"artag_create_marker.pgm");

if(argc>=4) label_on=1;
else        label_on=0;


printf("CREATE_ARTAG_MARKER.C\n");

/*****************************************************************************/
if((artag_id<0)||(artag_id>2047))
   {printf("ERROR: ARTag ID must be in range 0-2047 inclusive\n");exit(1);}

//check for black pattern
invert=0;
if(artag_id>1023) {artag_id-=1024;invert=1;}

if(label_on)
   {
   width=200;  height=200;
   xoffset=50; yoffset=50;
   }
else
   {
   width=200;  height=200;
   xoffset=50; yoffset=50;
   }
scale=10;

image=(unsigned char*)malloc(width*height);
if(image==NULL) {printf("Can't malloc image\n");exit(1);}

scale=10; 
pattern_width=10*scale;
pattern_image=(unsigned char*)malloc(pattern_width*pattern_width);
if(pattern_image==NULL) {printf("Can't malloc pattern_image\n");exit(1);}

//initialize character patterns for writing numbers 0-9 to image
if(label_on)
   {
   mgcd=(unsigned char*)malloc(2100);
   if(mgcd==NULL) {printf("Malloc mgcd error\n");exit(1);}
   artag_load_char_data();
   }

//clear to all white
for(i=0;i<width*height;i++) image[i]=255;

//call artag_create_marker() to get pattern
if(artag_create_marker(artag_id,scale,pattern_image)) 
   {printf("Error calling artag_create_marker()\n");exit(1);}

//copy pattern into *image    
for(j=0;j<pattern_width;j++)
   for(i=0;i<pattern_width;i++)
      image[i+xoffset+(j+yoffset)*width]=pattern_image[i+j*pattern_width];

if(label_on)
   {
   sprintf(message,"%d",artag_id+invert*1024);
   artag_big_print_string(image,width,height,width/2-strlen(message)*8,20,message,0,255);
   }

//invert codes 1024-2047 - only invert area around internal pattern
if(invert)
   for(j=0;j<height;j++) 
      for(i=0;i<width;i++) 
	     {
		 int celly=(j-yoffset)/scale,cellx=(i-xoffset)/scale;
		 if((cellx<2)||(cellx>7)||(celly<2)||(celly>7))  image[i+j*width]=255-image[i+j*width];
		 }

/*****************************************************************************/
//write to PGM file

sprintf(comment,"CREATE_ARTAG_MARKER.C from artag ID=%d",artag_id);
out=(FILE*)fopen(pgm_out_filename,"wb");
if(out==NULL)
   {printf("ERROR: Couldn't open %s for writing\n",pgm_out_filename);exit(1);}
fprintf(out,"P5\n#%s\n",comment);
fprintf(out,"%d %d\n255\n",width,height);
for(i=0;i<width*height;i++)
    {
    j=(int)(*(image+i));
    fputc(j,out);
    }
fclose(out);


printf("Wrote <%s>\n",pgm_out_filename);

return 0;
}





/****************************************************************************************************/
// numerical text: 0-9 characters double size font



/*artag_print_string returns the x after writing so draw_bit can start from there */
int artag_big_print_string(unsigned char *canvas, int canvas_width, int canvas_height,
                           int x, int y, char *s, int colour, int back_colour)
{
int i,c;

i=0;
while((c=*(s+i))!=0)
     {artag_big_print_char(canvas,canvas_width,canvas_height,x,y,c,colour,back_colour);x+=16;i++;}
return x;
}



void artag_big_print_char(unsigned char *canvas, int canvas_width, int canvas_height,
                          int x, int y, int ch, int colour, int back_colour)
{
int i,j,chh;
unsigned char dot,bit_mask,byte;
unsigned char *eximage_temp,*eximage_temp2;

if((ch<0x30)||(ch>0x39)) return;
ch+=0x10;

if((x>=0)&&(x+16<canvas_width)&&(y>=0)&&(y+16<canvas_height))
   {
      //can do it quickly without checking every pixel
   chh=(int)ch;
   eximage_temp=canvas+canvas_width*y+x;
   eximage_temp2=canvas+canvas_width*(y+1)+x;
   for(i=chh*8;i<(chh+1)*8;i++)
      {
      bit_mask=0x1;
      byte=((*(mgcd+i))<<4)|((*(mgcd+i))>>4);   /* switch nybbles */
      for(j=0;j<8;j++)
         {
         if(byte&bit_mask)     dot=colour;
         else                  dot=back_colour;
         *(eximage_temp+2*j)=dot;   *(eximage_temp+2*j+1)=dot;
         *(eximage_temp2+2*j)=dot;   *(eximage_temp2+2*j+1)=dot;
         bit_mask=bit_mask<<1;
         }
      eximage_temp+=canvas_width*2;    eximage_temp2+=canvas_width*2;
      }
   }
else
   {
   //check every pixel
   chh=(int)ch;
   for(i=0;i<8;i++)
      {
      int ii=i+chh*8;
      bit_mask=0x1;
      byte=((*(mgcd+ii))<<4)|((*(mgcd+ii))>>4);   /* switch nybbles */
      for(j=0;j<8;j++)
         {
         if((x+2*j>=0)&&(x+2*j+1<canvas_width)&&(y+2*i>=0)&&(y+2*i+1<canvas_height))
            {
            if(byte&bit_mask)     dot=colour;
            else                  dot=back_colour;
	    if((dot==colour)||(back_colour!=-1))
	       {
               *(canvas+x+2*j+  (y+2*i)*canvas_width)=dot;
               *(canvas+x+2*j+1+(y+2*i)*canvas_width)=dot;
               *(canvas+x+2*j+  (y+2*i+1)*canvas_width)=dot;
               *(canvas+x+2*j+1+(y+2*i+1)*canvas_width)=dot; 
	       }
	    }
         bit_mask=bit_mask<<1;
         }
      }
   }
}



void artag_load_char_data(void)
{
int i;

i= 64*8; *(mgcd+i++)=0xc3;*(mgcd+i++)=0x66;*(mgcd+i++)=0x67;*(mgcd+i++)=0xe6;
         *(mgcd+i++)=0x66;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;*(mgcd+i++)=0x00;
i= 65*8; *(mgcd+i++)=0x81;*(mgcd+i++)=0x81;*(mgcd+i++)=0xc1;*(mgcd+i++)=0x81;
         *(mgcd+i++)=0x81;*(mgcd+i++)=0x81;*(mgcd+i++)=0xe7;*(mgcd+i++)=0x00;
i= 66*8; *(mgcd+i++)=0xc3;*(mgcd+i++)=0x66;*(mgcd+i++)=0x06;*(mgcd+i++)=0x03;
         *(mgcd+i++)=0xc0;*(mgcd+i++)=0x60;*(mgcd+i++)=0xe7;*(mgcd+i++)=0x00;
i= 67*8; *(mgcd+i++)=0xc3;*(mgcd+i++)=0x66;*(mgcd+i++)=0x06;*(mgcd+i++)=0x83;
         *(mgcd+i++)=0x06;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;*(mgcd+i++)=0x00;
i= 68*8; *(mgcd+i++)=0x06;*(mgcd+i++)=0x07;*(mgcd+i++)=0x86;*(mgcd+i++)=0x66;
         *(mgcd+i++)=0xef;*(mgcd+i++)=0x06;*(mgcd+i++)=0x06;*(mgcd+i++)=0x00;
i= 69*8; *(mgcd+i++)=0xe7;*(mgcd+i++)=0x60;*(mgcd+i++)=0xe3;*(mgcd+i++)=0x06;
         *(mgcd+i++)=0x06;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;*(mgcd+i++)=0x00;
i= 70*8; *(mgcd+i++)=0xc3;*(mgcd+i++)=0x66;*(mgcd+i++)=0x60;*(mgcd+i++)=0xe3;
         *(mgcd+i++)=0x66;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;*(mgcd+i++)=0x00;
i= 71*8; *(mgcd+i++)=0xe7;*(mgcd+i++)=0x66;*(mgcd+i++)=0x03;*(mgcd+i++)=0x81;
         *(mgcd+i++)=0x81;*(mgcd+i++)=0x81;*(mgcd+i++)=0x81;*(mgcd+i++)=0x00;
i= 72*8; *(mgcd+i++)=0xc3;*(mgcd+i++)=0x66;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;
         *(mgcd+i++)=0x66;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;*(mgcd+i++)=0x00;
i= 73*8; *(mgcd+i++)=0xc3;*(mgcd+i++)=0x66;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc7;
         *(mgcd+i++)=0x06;*(mgcd+i++)=0x66;*(mgcd+i++)=0xc3;*(mgcd+i++)=0x00;
}



