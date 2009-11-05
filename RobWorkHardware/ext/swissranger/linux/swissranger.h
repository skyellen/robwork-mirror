/*
 * $Id: swissranger.h,v 1.1 2006-12-12 08:45:27 lpeu Exp $
 * Copyright © 2004, CSEM
 * All rights reserved.
 * Proprietary software. Use is subject to license terms as published in
 * http://www.csem.ch/fs/copyright.htm
 */

/*
 * This is the Linux, user-side driver for CSEM's Swiss Ranger 2
 * camera. The driver has been developed and tested under Linux 2.4.26
 * and Linux 2.6.10 and requires libusb-0.1.8, which itself requires
 * usbdevfs.
 *
 * Note that in order to talk to the Swiss Ranger 2 as a user other
 * than root, it is necessary to allow read and write access to the
 * device, which might not be the default mount setting. If you want
 * to access the camera as a user other than root, make sure you give
 * the appropriate permissions with chmod. For a permanent solution
 * modify /etc/fstab by setting devmode appropriately, for example:
 *
 * usbdevfs  /proc/bus/usb  usbdevfs  devmode=0666,noauto 0 0
 *
 * For other mount options, refer to the usbdevfs implementation.
 *
 *
 * @version $Revision: 1.1 $, $Date: 2006-12-12 08:45:27 $
 * @author Gabriel Gruener, gabriel.gruener@csem.ch
 */

/*
 * MODIFICATION HISTORY:
 *    $Log: swissranger.h,v $
 *    Revision 1.1  2006-12-12 08:45:27  lpeu
 *    Added SwissRanger to ext
 *
 *    Revision 1.1  2005/12/18 09:32:16  lpeu
 *    *** empty log message ***
 *
 *    Revision 1.3  2005/02/14 20:21:12  ggr
 *    Fixed documentation for swissranger_acquire.
 *
 *    Revision 1.2  2004/06/09 13:34:00  ggr
 *    Final version using libusb-0.1.8
 *
 *    Revision 1.1  2004/02/29 12:04:43  ggr
 *    Initial revision
 *
 */

#ifndef SWISSRANGER_H
#define SWISSRANGER_H


/***   Relevant constants   ***/

/* CSEM vendor ID */
#define SWISSRANGER_VENDOR_ID 0x0852

/* Swiss Ranger 2 device ID */
#define SWISSRANGER_DEVICE_ID 0x0071

/* Device interface to be claimed by this driver */
#define SWISSRANGER_INTERFACE_NUMBER 0

/* Number of rows in an image */
#define SWISSRANGER_ROWS 124

/* Number of columns in an image */
#define SWISSRANGER_COLUMNS 160



/*** Function prototypes ***/

/**
 * Travels the USB tree of connected devices and searches
 * for the Swiss Ranger 2 by matching the SWISSRANGER_VENDOR_ID and
 * SWISSRANGER_DEVICE_ID. If the Swiss Ranger 2 is found, the device
 * is opened, the interface SWISSRANGER_INTERFACE_NUMBER is claimed,
 * and an id of the opened device is returned.
 * If the device is not found, a negative error number is returned
 * and errno is set to the appropriate value.
 *
 * @return a device id (greater than zero) to the opened Swiss Ranger
 * device or a negative number in case of error, setting errno. In
 * particular, if the device is not found -ENODEV is returned. 
 */
int swissranger_open();

/**
 * Closes the Swiss Ranger 2 device, first releasing the claimed
 * interface.
 *
 * @param fd a valid device id returned by the swissranger_open
 * function.
 * @return a negative number in case of error; errno is set.
 */
int swissranger_close(int fd);

/**
 * Acquires one frame from the Swiss Ranger 2. The data will be placed
 * in the pixels argument. Note that this array should be big enough
 * to contain the number of frames expected from the camera, depending
 * on the mode setting (e.g. the default configuration mode returns a
 * distance frame and an intensity frame). Each frame will contain at
 * most SWISSRANGER_ROWS * SWISSRANGER_COLUMNS pixels. The actual
 * number of pixels in the frame depends on the Region Of Interest
 * setting of the camera. Each pixel requires 2 bytes.
 *
 * @param fd a valid device id returned by the swissranger_open
 * function.
 * @param pixels an allocated array of at least size length,
 * where the data from the camera will be placed.
 * @param length the number of bytes to read.
 * @return the actual number of bytes read or less than zero if an
 * error occurs; errno is set.
 */
int swissranger_acquire(int fd, void *pixels, int length);


/**
 * Sends parameter values to the camera's registers for configuration.
 * Refer to the camera's documentation for a list of available
 * registers and allowable values.
 *
 * @param fd a valid device id returned by the swissranger_open
 * function.
 * @param address the address of the descriptor to write to.
 * @param value the value to send to the given descriptor.
 * @return the number of bytes sent to the camera, which should be 2,
 * or a negative number if an error occurs; errno is set.
 */
int swissranger_send(int fd, char address, char value);


#endif /* SWISSRANGER_H */
