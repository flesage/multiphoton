# -*- coding: utf-8 -*-
"""
Created on Sat Sep 12 15:49:50 2015

@author: flesage
"""

import ctypes as C
import ctypes.util as Cutil
import threading
import numpy as np
import Queue

class IMAQError(Exception):
    """A class for errors produced during the calling of National Intrument's IMAQ functions.
    It will also produce the textual error message that corresponds to a specific code."""

    def __init__(self, code):
        self.code = code
        text = C.c_char_p('')
        NiIMAQCamera.imaq.imgShowError(code, text)
        self.message = "{}: {}".format(self.code, text.value)
        # Call the base class constructor with the parameters it needs
        Exception.__init__(self, self.message)


def imaq_error_handler(code):
    """Print the textual error message that is associated with the error code."""

    if code < 0:
        raise IMAQError(code)
        free_associated_resources = 1
        NiIMAQCamera.imaq.imgSessionStopAcquisition(NiIMAQCamera.sid)
        NiIMAQCamera.imaq.imgClose(NiIMAQCamera.sid, free_associated_resources)
        NiIMAQCamera.imaq.imgClose(NiIMAQCamera.iid, free_associated_resources)
    else:
        return code


class NiIMAQCamera():

    _IMG_BASE = 0x3FF60000
    IMG_ATTR_ACQWINDOW_HEIGHT = _IMG_BASE + 0x006B
    IMG_ATTR_ROI_WIDTH = _IMG_BASE + 0x01A6
    IMG_ATTR_ROI_HEIGHT = _IMG_BASE + 0x01A7
    IMG_ATTR_BYTESPERPIXEL = _IMG_BASE + 0x0067
    IMG_ATTR_PIXDEPTH = _IMG_BASE + 0x0002  # pix depth in bits
    IMG_ATTR_BITSPERPIXEL = _IMG_BASE + 0x0066 # aka the bit depth
    IMG_BUFF_ADDRESS = _IMG_BASE + 0x007E  # void *
    IMG_BUFF_COMMAND = _IMG_BASE + 0x007F  # uInt32
    IMG_BUFF_SIZE = _IMG_BASE + 0x0082  #uInt32
    IMG_CMD_STOP = 0x08  # single shot acquisition
    PULSE_TIMEBASE_PIXELCLK = 0x00000001
    IMG_SIGNAL_STATUS = 4
    IMG_LINE_VALID = 18
    IMG_FRAME_DONE = 10
    IMG_TRIG_POLAR_ACTIVEH = 0
    IMG_SIGNAL_EXTERNAL= 0
    IMG_SIGNAL_RTSI= 1
    IMG_PULSE_POLAR_ACTIVEH = 0
    IMG_TRIG_POLAR_ACTIVEL = 1
    PULSE_MODE_SINGLE_REARM = 0x00000003
    IMG_EXT_TRIG0 = 0
    imaqlib_path = Cutil.find_library('imaq')
    imaq = C.windll.LoadLibrary(imaqlib_path)
    iid = C.c_uint32(0)
    sid = C.c_uint32(0)

    def __init__(self):


        imaq_function_list = [  # functions used in this program
            self.imaq.imgGetAttribute,
            self.imaq.imgInterfaceOpen,
            self.imaq.imgSessionOpen,
            self.imaq.niimaquDisable32bitPhysMemLimitEnforcement,  # because we're running on a 64-bit machine with over 3GB of RAM
            self.imaq.imgCreateBufList,
            self.imaq.imgCreateBuffer,
            self.imaq.imgSetBufferElement2,
            self.imaq.imgGrabSetup,
            self.imaq.imgSessionSaveBufferEx,
            self.imaq.imgSessionStopAcquisition,
            self.imaq.imgClose,
            self.imaq.imgSetCameraAttributeNumeric,
            self.imaq.imgPulseCreate2,
            self.imaq.imgSessionLineTrigSource2
            ]

        # for all imaq functions we're going to call, we should specify that if they
        # produce an error (a number), we want to see the error message (textually)
        for func in imaq_function_list:
            func.restype = imaq_error_handler

        self.consumers=[]

    def setDataConsumer(self,consumer, wait):

        # Consumers list contains triplet of consumer, push method and channel to push
        self.consumers.append(consumer)
        self.consumers.append(wait)

    def acquire(self):

        while self.started == True:

            #Get the frame after the next Vertical Blank
            self.imaq.imgGrab(self.sid, self.imgbuffer_vpp, 1)
            tmp = np.ctypeslib.as_array(self.imgbuffer, shape=(1024*self.n_lines,))
            img=np.reshape(tmp,(self.n_lines,1024))
            for ic in range(0,len(self.consumers),2):
                try:
                    self.consumers[ic].put(img.astype('int16'),self.consumers[ic+1])
                except Queue.Full:
                    pass

    def open_camera(self,camera_name,n_lines,exposure):


        # OPEN A COMMUNICATION CHANNEL WITH THE CAMERA
        # our camera has been given its proper name in Measurement & Automation Explorer (MAX)
        self.imaq.imgInterfaceOpen(camera_name, C.byref(self.iid))
        self.imaq.imgSessionOpen(self.iid, C.byref(self.sid))
        self.n_lines=n_lines
        tmp = C.c_uint32(int(self.n_lines))
        self.imaq.imgSetAttribute2(self.sid, NiIMAQCamera.IMG_ATTR_ACQWINDOW_HEIGHT, tmp)
        self.imaq.imgSessionLineTrigSource2(self.sid,NiIMAQCamera.IMG_SIGNAL_EXTERNAL,NiIMAQCamera.IMG_EXT_TRIG0,NiIMAQCamera.IMG_TRIG_POLAR_ACTIVEL)

        width, height = C.c_uint32(), C.c_uint32()
        pixdepth, bitsperpixel, bytes_per_pixel = [ C.c_uint8() for _ in range(3) ]
        # poll the camera (or is it the camera file (icd)?) for these attributes and store them in the variables
        for var, macro in [ (width, NiIMAQCamera.IMG_ATTR_ROI_WIDTH),
                           (height, NiIMAQCamera.IMG_ATTR_ROI_HEIGHT),
                            (bytes_per_pixel, NiIMAQCamera.IMG_ATTR_BYTESPERPIXEL),
                            (pixdepth, NiIMAQCamera.IMG_ATTR_PIXDEPTH),
                            (bitsperpixel, NiIMAQCamera.IMG_ATTR_BITSPERPIXEL) ]:
            self.imaq.imgGetAttribute(self.sid, macro, C.byref(var))
        e_time=C.c_double(exposure)
        self.imaq.imgSetCameraAttributeNumeric(self.sid, 'Exposure Time', e_time)
        self.bufsize = width.value*height.value*bytes_per_pixel.value
        self.imaq.niimaquDisable32bitPhysMemLimitEnforcement(self.sid)

    def open_2d_camera(self,camera_name,exposure):

        # OPEN A COMMUNICATION CHANNEL WITH THE CAMERA
        # our camera has been given its proper name in Measurement & Automation Explorer (MAX)
        self.imaq.imgInterfaceOpen(camera_name, C.byref(self.iid))
        self.imaq.imgSessionOpen(self.iid, C.byref(self.sid))

        width, height = C.c_uint32(), C.c_uint32()
        pixdepth, bitsperpixel, bytes_per_pixel = [ C.c_uint8() for _ in range(3) ]
        # poll the camera (or is it the camera file (icd)?) for these attributes and store them in the variables
        for var, macro in [ (width, NiIMAQCamera.IMG_ATTR_ROI_WIDTH),
                           (height, NiIMAQCamera.IMG_ATTR_ROI_HEIGHT),
                            (bytes_per_pixel, NiIMAQCamera.IMG_ATTR_BYTESPERPIXEL),
                            (pixdepth, NiIMAQCamera.IMG_ATTR_PIXDEPTH),
                            (bitsperpixel, NiIMAQCamera.IMG_ATTR_BITSPERPIXEL) ]:
            self.imaq.imgGetAttribute(self.sid, macro, C.byref(var))
        e_time=C.c_double(exposure)
        self.imaq.imgSetCameraAttributeNumeric(self.sid, 'Exposure Time', e_time)
        self.bufsize = width.value*height.value*bytes_per_pixel.value
        self.imaq.niimaquDisable32bitPhysMemLimitEnforcement(self.sid)

    def configure_for_single_grab(self):
        # create a single buffer (in a list)
        bid = C.c_uint32(0)
        self.imaq.imgCreateBufList(1, C.byref(bid))
        # Let NI-IMAQ manage the memory
        # CONFIGURE THE PROPERTIES OF THE BUFFER
        self.imgbuffer = C.POINTER(C.c_uint16)()  # create a null pointer
        self.imaq.imgCreateBuffer(self.sid, 0, self.bufsize, C.byref(self.imgbuffer))  # allocate memory (the buffer) on the host machine (param2==0)
        self.imaq.imgSetBufferElement2(bid, 0, NiIMAQCamera.IMG_BUFF_ADDRESS,self.imgbuffer)  # my guess is that the cast to an uint32 is necessary to prevent 64-bit callable memory addresses
        self.imaq.imgSetBufferElement2(bid, 0, NiIMAQCamera.IMG_BUFF_SIZE, self.bufsize)
        self.imaq.imgSetBufferElement2(bid, 0, NiIMAQCamera.IMG_BUFF_COMMAND, NiIMAQCamera.IMG_CMD_STOP)
        self.imgbuffer_vpp = C.cast(C.byref(self.imgbuffer), C.POINTER(C.c_void_p))
        self.imaq.imgGrabSetup (self.sid, 1)

    def start(self):
        # Thread to display so that the puts in queue are very fast
        self.t1 = threading.Thread(target=self.acquire)
        self.started = True
        self.t1.start()

    def stop(self):
        self.started = False
        self.t1.join()

    def close_camera(self):
        free_associated_resources = 1
        self.imaq.imgSessionStopAcquisition(self.sid)
        self.imaq.imgClose(self.sid, free_associated_resources )
        self.imaq.imgClose(self.iid, free_associated_resources )