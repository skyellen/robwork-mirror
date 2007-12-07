//ARTAG_DETECT_IN_IMAGE.C  - handles PGM and PPM files
//July 12/2004 - Mark Fiala, National Research Council of Canada

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "artag_rev1.h"

#define MAX_NUM_MARKERS 1000

//prototypes
unsigned char* artagdet_read_pgm(char *file_name, int *width, int *height);
unsigned char* artagdet_read_ppm(char *file_name, int *width, int *height);
void artagdet_endpoints2line(double x0, double y0, double x1, double y1, 
                             double *a, double *b, double *c);
int artagdet_lines2intersection(double a1, double b1, double c1,
                                double a2, double b2, double c2,
		                        double *x, double *y);
char artagdet_split_filename(char *filename, char *root, char *suffix);




int main(int argc, char *argv[])
{
unsigned char *image;
char image_in_filename[256];
int i,thresh,num_markers_found;
int width,height;
char rgb_greybar;
char root[256],suffix[256];
int artag_pattern_id[MAX_NUM_MARKERS];
double artag_pattern_x0[MAX_NUM_MARKERS],artag_pattern_y0[MAX_NUM_MARKERS];
double artag_pattern_x1[MAX_NUM_MARKERS],artag_pattern_y1[MAX_NUM_MARKERS];
double artag_pattern_x2[MAX_NUM_MARKERS],artag_pattern_y2[MAX_NUM_MARKERS];
double artag_pattern_x3[MAX_NUM_MARKERS],artag_pattern_y3[MAX_NUM_MARKERS];


if((argc!=2)&&(argc!=3))
   {printf("ARTAG_DETECT_IN_IMAGE.C finds ARTag markers in a PGM or PPM file\n"
           "-needs 1 argument\n"
           "-proper format is: ARTAG_DETECT_IN_IMAGE input.pgm \n"
           "-output is displayed to the command line.\n"
	       );
    exit(1);}
printf("ARTAG_DETECT_IN_IMAGE.C    ");


//init_vgatext();
strcpy(image_in_filename,argv[1]);

if(argc==3) sscanf(argv[2],"%d",&thresh);
else        thresh=100;

artagdet_split_filename(image_in_filename,root,suffix);
if((strcmp(suffix,"pgm")==0)||(strcmp(suffix,"PGM")==0))       rgb_greybar=0;
else if((strcmp(suffix,"ppm")==0)||(strcmp(suffix,"PPM")==0))  rgb_greybar=1;
else {printf("ERROR: Can only process PGM or PPM image files\n");exit(1);}

/*****************************************************************************/
// -- Read in image --
if(rgb_greybar==0)  image=artagdet_read_pgm(image_in_filename,&width,&height);
else                image=artagdet_read_ppm(image_in_filename,&width,&height);
if(image==NULL)  {printf("Can't load input image file <%s>\n",image_in_filename);exit(1);}

/*****************************************************************************/
init_artag(width,height,0);  //last argument dosen't matter in non-ARToolkit plugin apps

// detect ARtag markers
artag_find_marker(image,rgb_greybar,artag_pattern_id,
                  artag_pattern_x0,artag_pattern_y0,artag_pattern_x1,artag_pattern_y1,
                  artag_pattern_x2,artag_pattern_y2,artag_pattern_x3,artag_pattern_y3,
                  &num_markers_found);

printf("%d ARTag markers detected for a threshold of %d\n",num_markers_found,thresh);
for(i=0;i<num_markers_found;i++)
   {
   int i0=(int)artag_pattern_x0[i];
   int j0=(int)artag_pattern_y0[i];
   int i1=(int)artag_pattern_x1[i];
   int j1=(int)artag_pattern_y1[i];
   int i2=(int)artag_pattern_x2[i];
   int j2=(int)artag_pattern_y2[i];
   int i3=(int)artag_pattern_x3[i];
   int j3=(int)artag_pattern_y3[i];
   double a1,b1,c1,a2,b2,c2,xcenter,ycenter;
   artagdet_endpoints2line((double)i0,(double)j0,(double)i2,(double)j2,&a1,&b1,&c1);
   artagdet_endpoints2line((double)i1,(double)j1,(double)i3,(double)j3,&a2,&b2,&c2);
   if(artagdet_lines2intersection(a1,b1,c1,a2,b2,c2,&xcenter,&ycenter)!=0) {xcenter=-100.0;ycenter=-100.0;}
   printf("ARTag marker %d found with center at (%d,%d)\n",artag_pattern_id[i],
          (int)xcenter,(int)ycenter);
   printf("                corners (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",i0,j0,i1,j1,i2,j2,i3,j3);
   }


return 0;
}





/*****************************************************************************/
// Functions


char artagdet_split_filename(char *filename, char *root, char *suffix)
{
int i;
//find .
i=strlen(filename)-1;
if(i<3) return -1;
while((filename[i]!='.')&&(i>=0)) i--;
//{printf("%d <%c>\n",i,filename[i]);i--;}
if(i==0) return -1;
//make root and suffix
strcpy(root,filename);
root[i]=0;
strcpy(suffix,&(filename[i+1]));

return 0;
}



unsigned char* artagdet_read_pgm(char *file_name, int *width, int *height)
{
int i,size;
int max_grey;
FILE *in_file;
char c,comment[256];
unsigned char uc,*image;

in_file=fopen(file_name,"rb");
if(in_file==NULL)  return NULL;

c=fgetc(in_file); if(c!='P') {printf("%s Not a PGM file\n",file_name);exit(1);}
c=fgetc(in_file); if(c!='5') {printf("%s Not a PGM file\n",file_name);exit(1);}
c=fgetc(in_file); if(c!='\n') {printf("%s Not a PGM file\n",file_name);exit(1);}
while(1)
    {
    fgets(comment,256,in_file);
    if(comment[0]!='#') break;
    }
sscanf(comment,"%d %d",width,height);
fgets(comment,256,in_file);
sscanf(comment,"%d",&max_grey);

size=(*width)*(*height);
image=(unsigned char*)malloc(size);
if(image==NULL) {printf("artagdet_read_pgm() error: Couldn't malloc image\n");exit(1);}

for(i=0;i<size;i++)
   {
   uc=fgetc(in_file);
   *(image+i)=uc;
   }
fclose(in_file);
return image;
}




unsigned char* artagdet_read_ppm(char *file_name, int *width, int *height)
{
int i,size;
int max_grey;
FILE *in_file;
char c,comment[256]; 
unsigned char uc,*image;

in_file=fopen(file_name,"rb");
if(in_file==NULL)  return NULL;

c=fgetc(in_file); if(c!='P') {printf("%s Not a PPM file\n",file_name);exit(1);}
c=fgetc(in_file); if(c!='6') {printf("%s Not a PPM file\n",file_name);exit(1);}
c=fgetc(in_file); if(c!='\n') {printf("%s Not a PPM file\n",file_name);exit(1);}
while(1)
    {
    fgets(comment,256,in_file);
    if((comment[0]!='#')&&(comment[0]!=0x0a)&&(comment[0]!=0x0d)) break;
    }
sscanf(comment,"%d %d",width,height);
fgets(comment,256,in_file);
sscanf(comment,"%d",&max_grey);

size=(*width)*(*height);
image=(unsigned char*)malloc(size*3+10);
if(image==NULL) {printf("artagdet_read_ppm() error: Couldn't malloc image\n");exit(1);}

for(i=0;i<size*3;i++)
   {
   uc=fgetc(in_file);
   *(image+i)=uc;
   }
fclose(in_file);
return image;
}



//  artagdet_endpoints2line(x0,y0,x1,y1,&a,&b,&c);
void artagdet_endpoints2line(double x0, double y0, double x1, double y1, double *a, double *b, double *c)
{
*a=y0-y1; *b=x1-x0; *c=x0*y1-x1*y0;
}


//  artagdet_lines2intersection(a1,b1,c1,a2,b2,c2,&x,&y);
int artagdet_lines2intersection(double a1, double b1, double c1,
                       double a2, double b2, double c2,
		       double *x, double *y)
{
double denom=a1*b2-a2*b1;
if(fabs(denom)<0.0001) return -1;
*x=(b1*c2-b2*c1)/denom;
*y=(a2*c1-a1*c2)/denom;
return 0;
}
