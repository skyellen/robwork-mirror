/*
 * $Id: swissranger.c,v 1.1 2006-12-12 08:45:27 lpeu Exp $
 * Copyright © 2004, CSEM
 * All rights reserved.
 * Proprietary software. Use is subject to license terms.
 */

/*
 * This is the Linux, user-side driver for CSEM's Swiss Ranger camera.
 * Refer to swissranger.h for complete documentation.
 *
 * @version $Revision: 1.1 $, $Date: 2006-12-12 08:45:27 $
 * @author Gabriel Gruener, gabriel.gruener@csem.ch
 */

/*
 * MODIFICATION HISTORY:
 *    $Log: swissranger.c,v $
 *    Revision 1.1  2006-12-12 08:45:27  lpeu
 *    Added SwissRanger to ext
 *
 *    Revision 1.1  2005/12/18 09:32:15  lpeu
 *    *** empty log message ***
 *
 *    Revision 1.5  2005/02/14 20:22:44  ggr
 *    Fixed return value of swissranger_acquire.
 *    Added error checking in swissranger_close.
 *
 *    Revision 1.4  2004/08/18 17:48:43  ggr
 *    Added capability to handle more than 1 device
 *
 *    Revision 1.3  2004/06/09 13:33:27  ggr
 *    Final version using libusb-0.1.8
 *
 *    Revision 1.2  2004/05/27 08:29:10  ggr
 *    Final version using ioctl calls.
 *
 *    Revision 1.1  2004/02/29 12:04:15  ggr
 *    Initial revision
 *
 */

/* TODO:
 *    Use a struct to describe the device, including file descriptor,
 *    buffers, USB transfer structures, etc.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <usb.h>

#include "swissranger.h"
#define __FUNCTION__ "Function"

/* A default timeout for all USB functions */
#define	TIMEOUT	(1 * 1000)		// 1 second (in ms.)


#ifdef SWISSRANGER_DEBUG

#undef dbg
#define dbg(format, arg...) do { fprintf(stderr, __FILE__ ": " __FUNCTION__ ": " format "\n" , ## arg); } while (0)

#else

#undef dbg
#define dbg(format, arg...) do {} while (0)

#endif // SWISSRANGER_DEBUG


// Handle to the Swiss Ranger USB device
usb_dev_handle* swissranger_handle;


int swissranger_open() {

  // Usb device structure
  struct usb_device * dev;

  // Array of usb busses
  struct usb_bus * busses;

  // Pointer to current bus
  struct usb_bus * bus;

  // Stores return values from functions called
  int res;

  // Device driver name
  int driverNameLength = 50;
  char driverName [driverNameLength];


  // Clear handle
  swissranger_handle = NULL;

  // Initialize USB library
  usb_init();

  if (usb_find_busses() < 0) {

	dbg("Couldn't find any USB busses");
	return -errno;
  }

  if (usb_find_devices() < 0) {

	dbg("Couldn't find any USB devices");
	return -errno;
  }

  // Get the array of busses
  busses = usb_get_busses();

  bus = busses;
  while ((bus != NULL) && (swissranger_handle == NULL)) {

    dbg("USB bus found: %s", bus->dirname);

    dev = bus->devices;
    while ((dev != NULL) && (swissranger_handle == NULL)) {

	  dbg("  USB device found: %s", dev->filename);

      if ((dev->descriptor.idVendor == SWISSRANGER_VENDOR_ID) && (dev->descriptor.idProduct == SWISSRANGER_DEVICE_ID)) {

		dbg("    Swiss Ranger 2 found!");

		swissranger_handle = usb_open(dev);
		if (swissranger_handle == NULL) {

		  dbg ("    Couldn't open device with usb_open!");
		  return -ENODEV;
		}

		res = usb_get_driver_np(swissranger_handle, SWISSRANGER_INTERFACE_NUMBER, driverName, driverNameLength);

		if (res < 0) {  // If this happens assume device is free

		  res = usb_detach_kernel_driver_np(swissranger_handle, SWISSRANGER_INTERFACE_NUMBER);
		  res = usb_claim_interface(swissranger_handle, SWISSRANGER_INTERFACE_NUMBER);
		  if (res < 0) {

			dbg("    Error claiming interface %d of Swiss Ranger!", SWISSRANGER_INTERFACE_NUMBER);

			if (usb_close(swissranger_handle) < 0) {

			  swissranger_handle = NULL;
			  return -errno;
			}
			swissranger_handle = NULL;

		  } else {

			// We can't access the device's file descriptor
			// So make up a 'meaningful' one.
			/* return swissranger_handle->fd; */
			res = atoi(bus->dirname)*64 + atoi(dev->filename);
			return res > 0 ? res : dev->descriptor.idProduct;
		  }
		}
		if (usb_close(swissranger_handle) < 0) {

		  swissranger_handle = NULL;
		  return -errno;
		}
		swissranger_handle = NULL;
      }
      dev = dev->next;
    }
    bus = bus->next;
  }

  dbg("Finished reading USB device tree");

  // Didn't find Swiss Ranger device
  errno = ENODEV;
  return -errno;
}


int swissranger_close(int fd) {

  // We can't access the device's file descriptor
  // so don't check...
  /* 	if (fd != swissranger_handle->fd) {

  errno = EBADF;
  return -errno;
  }
  */

  // Release claimed interface of the device
  if (usb_release_interface(swissranger_handle, SWISSRANGER_INTERFACE_NUMBER) < 0) {

	dbg("Error releasing interface %d!", SWISSRANGER_INTERFACE_NUMBER);
	return -errno;
  }

  // Close the device
  dbg("Closing Swiss Ranger device");
  if (usb_close(swissranger_handle) < 0) {

	dbg("Error closing device");
	return -errno;
  }
  swissranger_handle = NULL;
  return 0;
}


int swissranger_acquire(int fd, void *pixels, int length) {

  // Request type: to device, vendor, from device
  int requesttype = 0x00 | 0x40 | 0x00;

  // Request: frame request
  int request = 0xB0;

  // Bulk read end-point
  int ep = 0x88;

  // Response from functions
  int res;

  // Fix length to a multiple of 512 bytes
  // TODO: Allocate this buffer only once in open
  //      then release it in close?
  int bufferLength = (length + 511) & (-512);
  char buffer [bufferLength];

  // Request one frame
  dbg("Requesting one frame");
  if (usb_control_msg(swissranger_handle, requesttype, request, 0, 0, NULL, 0, TIMEOUT) < 0) {

	dbg("Error requesting data frame from Swiss Ranger!");
	return -errno;
  }

  // Read the frame
  dbg("Attempting to read frame");
  if ((res = usb_bulk_read(swissranger_handle, ep, buffer, bufferLength, TIMEOUT)) < 0) {

	dbg("Error in bulk read!");
	return -errno;
  }

  dbg("Finished reading frame");

  // Copy buffer to user's array
  dbg("Attempting to copy data to user");
  pixels = memcpy(pixels, buffer, length);
  return (res < length) ? res : length;
}


int swissranger_send(int fd, char address, char value) {

  int res = 0;

  // Bulk write endpoint
  int ep = 0x02;

  int length = 2;
  char buffer [length];

  buffer[0] = address;
  buffer[1] = value;

  dbg("Attempting to write configuration");

  if ((res = usb_bulk_write(swissranger_handle, ep, buffer, length, TIMEOUT)) < 0) {

	dbg("Error in bulk write!");
	return -errno;
  }
  dbg("Finished writing configuration");
  return res;
}
