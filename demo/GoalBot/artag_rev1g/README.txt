ARTAG_REV1:  Linux Package  - July 2004 by Mark Fiala, of the National Research Council of Canada
By using this you agree to use it only for non-commercial purposes such as research, otherwise
please contact us (info at the bottom).
------------------------------------------------------------------------------------------------------
//-free for non-commercial uses such as research and education
//-this evaluation version times out after Dec 31/2006 - newer evaluation versions will be periodically available from
//the website www.artag.net


Mark
mark.fiala@nrc-cnrc.gc.ca

-----------------------------------------------------------------------------------------------------------
Where you got this from:
http://www.cv.iit.nrc.ca/research/ar/downloads.html or http://www.cs.ualberta.ca/~fiala/artag/
or http://www.artag.net

-----------------------------------------------------------------------------------------------------------
Welcome to ARTag!

ARTag is a marker detection system inspired by ARToolkit, the popular system designed for Augmented
Reality.  ARTag consists of a library of patterns, which when mounted on a flat surface and viewed by
the software, are detected and identified.  The four corner locations are provided and can be used to
determine the relative pose between a camera and a marker.  In practical terms this means a tea pot
or snowman can hover above a cardboard pattern as you wave it front of your webcam or look at it
with your HMD.

ARTag has improved (lower) false positive rate and inter-marker confusion rates for detecting
patterns.  It uses digital encoding methods instead of correlation as in ARToolkit.  This results
in an almost zero rate of falsely reporting a marker when it's not there, or confusing one marker
for another.  Thus far I have yet to see a false positive or inter-marker confusion event occur at
all in any of my programs.  ARTag also uses a different method to find 4-sided shapes in the
image removing the need for a threshold greyscale level.

ARTag uses square marker patterns as does ARToolkit, however it differs both in how it detects
quadrilaterals and how it encodes the marker.  ARTag finds quadrilaterals by linking edge pixels whereas
ARToolkit finds contours in a thresholded binary image.  ARTag does not require a threshold and can
tolerate some occlusion such as a broken side of a marker, or the image border occluding a corner.

ARTag has a library of 2002 markers.  No pattern files need to be loaded (as with ARToolkit).  Each
one has a number 0-2047, with 46 codes not to be used (682 and 1706 are degenerate cases in the digital
algebra used, others are not recommended due to smaller hamming distances between codes).
Patterns 0-1023 use a black square border on a white background, patterns 1024-2047 use a white square
border on a black background.  You can use the markers in the patterns/ directory, create ones using the
command line program <artag_create_marker.exe>, or use the artag_create_marker() function in one of your
programs to create the patterns.

Summary to including ARTag in your programs:
Case A: Existing ARToolkit program:
 1-include "artag_rev1.h", uncomment prototype for arDetectMarker() in "artag_rev1.h".
 2-Call init_artag(width,height,bpp); at the start of your program (bpp=1,3,or 4)
 3-Replace calls to ARToolkit's arDetectMarker() with artagDetectMarker(), keeping the same arguments
 4-Link in library <libartag_rev1.a> for C programs or <libartag_rev1_cpp.a> for CPP

Case B: Writing a new program:
 1-include "artag_rev_1.h", prototype for arDetectMarker() in "artag_rev1.h" should be commented out.
 2-Call init_artag(width,height,bpp); at the start of your program, bpp will be ignored in this case
 3-Call artag_find_marker() passing it your unsigned char *image
 4-Link in library <libartag_rev1.a> for C programs or <libartag_rev1_cpp.a> for CPP

There are two ways to use this package, either in one of your programs or to "drop in" to your existing
ARToolkit program.  If you write a new program from scratch, it's recommended you call the
artag_find_marker() function to keep things simple.

If you are an existing ARToolkit user, and just want to drop in ARTag, call the artagDetectMarker()
function in place of the existing line in your program that calls arDetectMarker().  You can use the
same arguments and it will return the data in the ARMarkerInfo structure format, you should just be able
to add the letters "tag" into the function call and it should work right away.  If you do this, you need
to also uncomment the prototype for artagDetectMarker() in <artag_rev1.h>.

If you are modifying an existing ARToolkit program, you can either leave the calls to arLoadPatt(patt_name)
or replace them with the drop-in function artagLoadPatt(patt_name), the latter is recommended for increased
hamming distances between marker codes.  Which you do will select which marker ID's will substitute for the
original ARToolkit patterns.  With the former, you would use ARTag patterns ID#0,1,2,... (since that is the
order artagLoadPatt() returns pattern handles).  If you only have a small number of patterns, it would be
simpler to just leave the ARToolkit arLoadPatt(patt_name) call in and use ARTag patterns #0,1,2,etc, it
will unnecessarily load the ARToolkit patterns but will save programming time.  If you replace the ARToolkit
pattern loading function call with the ARTag drop in; patt_id=artagLoadPatt(patt_name), you will get
non-sequential ARTag ID's; #4,57,260,5,... which are the first few ARTag ID's from the recommended list.
This function calls the artag_get_id() function described below.  You would use this approach if you have
many markers and/or want to eliminate having the ARToolkit pattern files present.  It is advised that you
comment out the artagLoadPatt() calls after the first time you run your program, since each call to the
function creates or appends a file <artag_marker_mappings.txt>.  This text file is useful to allow you to
correlate an ASCII string with each marker pattern.  The first 18 consecutive patterns are combined in
patterns/panel_0_17.pgm, and the first 18 recommended patterns are combined in patterns/panel_18_best.pgm.

Any set of ARTag markers will probably work for you, but if you want to get the best immunity from inter-
marker confusion, use the artag_get_id() function to get ARTag ID numbers.  Every time you call this it will
pull a marker ID off of the recommended list (in an order reset when init_artag is called).  Look further down
at the --Recommended Marker Sets-- section for more info.

Here's how to compile the two example .c files provided;
gcc -o artag_create_marker artag_create_marker.c -L. -lartag_rev1 -lm
gcc -o artag_detect_in_image artag_detect_in_image.c -L. -lartag_rev1 -lm

-----------------------------------------------------------------------------------------------------------
What this ARTAG_REV1.TGZ files contains:

(sorry, no modified ARToolkit examples for Linux/Unix, please see the Windows version of Rev1)

2 command line programs with source .c files
1 GUI program without source

2 command line programs programs <artag_detect_in_image> and <artag_create_marker>, run them without arguments
and it will tell you the format

1 GUI program <artag_dragonfly>. Run this if you have a "Dragonfly" IEEE1394 camera from Point
Grey Research (http://www.ptgrey.com).  It highlights the detected markers and you can see the
processing time.


-----------------------------------------------------------------------------------------------------------
--Recommended Marker Sets--
You can minimize the (already low) inter-marker confusion rate by choosing markers in order from a pre-
calcluated list.  For example, if your application needs 50 markers, take artag_recommended[0] through
artag_recommended[49].  You can find an explanation in my publications.
The artag_get_id(width,height,name) pulls marker numbers from the recommended set building list.  This list is
found in the artag_recommended[] statically defined structure.  The first time you call it, it gives you
artag_recommended[0] which is 4, the next time you call it, it gives you artag_recommended[1]=57,
260,5,52,59,...  You can reset it by setting the global artag_recommended_marker_pointer back to zero.

-----------------------------------------------------------------------------------------------------------
Commercial uses:  This was developed for research purposes, but you can use it commercially if you obtain
our permission.

If you download ARToolkit and modify it using our library, please contact them as well if you
wish to use any of that code commercially.

You cannot use the lartag_revX.a library file (where X is the revision number) commercially
without our permission otherwise you'll be breaking the downloading agreement.

To contact the NRC if you wish to use it commercially, please contact either myself (mark.fiala@nrc-cnrc.gc.ca)
or a Business Development Officer from our institute (IIT).
http://iit-iti.nrc-cnrc.gc.ca/about-sujet/bdo-bdc_e.html

NRC Institute for Information Technology
1200 Montreal Road, Building M-50
Ottawa, ON  K1A 0R6  Canada



