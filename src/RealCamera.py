"""
Script defining Camera class and uses Image class. Enables the use of IDS cameras, and processing of 
taken images. 

Author: Raul Penagos
Date: Feb 13th, 2025
"""

import ids_peak.ids_peak as ids_peak
import ids_peak_ipl.ids_peak_ipl as ids_ipl
import ids_peak.ids_peak_ipl_extension as ids_ipl_extension
from src.Image import Image
import numpy as np

class IDS_Camera:
    """
    Class that enables the conection with an IDS industrial camera by creating an instance of it.
    Enables changing exposure_time, take images process and save them as Image instances.
    For using a camera:
        0 - Plug camera USB to PC
        1 - Run start_camera (if cameras are available).
        2 - (Optional) Use set_exposure().
        3 - Use auto_exposure_get_image() or get_image() to take and save pictures.
    """
    def __init__(self):

        self.device_descriptors = None
        self.device = None
        self.remote_device_nodemap = None
        self.datastream = None
        self.exposure_time_seg = 1/250

        self.name = None
        self.image = None

    def _search_device(self):
        """
        Searches for devices compatible with IDS industrial cameras
        """
        try:
            ids_peak.Library.Close()
            ids_peak.Library.Initialize()
            device_manager = ids_peak.DeviceManager.Instance()
            device_manager.Update()
            self.device_descriptors = device_manager.Devices()

            print("Found Devices: " + str(len(self.device_descriptors)))

            for self.device_descriptor in self.device_descriptors:
                print(self.device_descriptor.DisplayName())

            return self
        except Exception as e:
            print('ERR:' + str(e))
            ids_peak.Library.Close()
            
    def _open_device(self):
        """
        Opens available devices.
        Will give an error if the devices are already in use
        """
        try:
            self.device = self.device_descriptors[0].OpenDevice(ids_peak.DeviceAccessType_Control)
            print("Opened Device: " + self.device.DisplayName())
            self.remote_device_nodemap = self.device.RemoteDevice().NodeMaps()[0]

            # Set Software trigger: Single frame acquisition
            self.remote_device_nodemap.FindNode("TriggerSelector").SetCurrentEntry("ExposureStart")
            self.remote_device_nodemap.FindNode("TriggerSource").SetCurrentEntry("Software")
            self.remote_device_nodemap.FindNode("TriggerMode").SetCurrentEntry("On")

            return self
        except Exception as e:
            print('No device is free and available. ERR:' + str(e))
            ids_peak.Library.Close()

    def _start_acquisition(self):
        """
        Starts acquisition time, during this time Images can be taken
        """
        try:
            self.datastream = self.device.DataStreams()[0].OpenDataStream()
            payload_size = self.remote_device_nodemap.FindNode("PayloadSize").Value()
            for i in range(self.datastream.NumBuffersAnnouncedMinRequired()):
                buffer = self.datastream.AllocAndAnnounceBuffer(payload_size)
                self.datastream.QueueBuffer(buffer)

            self.datastream.StartAcquisition()
            self.remote_device_nodemap.FindNode("AcquisitionStart").Execute()
            self.remote_device_nodemap.FindNode("AcquisitionStart").WaitUntilDone()

            return self
        except Exception as e:
            print('No device is free and available. ERR:' + str(e))
            ids_peak.Library.Close()

    def start_camera(self):
        """
        Initializes and opens te camera in one step
        """
        self._search_device()
        self.name = self.device_descriptor.DisplayName()
        self._open_device()

    def set_exposure(self, exposure_time_seg = 1/250):
        """
        Sets exposure time for the capture
        """
        try: 
            self.exposure_time_seg = exposure_time_seg
            exposure_time_microseg = exposure_time_seg * 1e6
            self.remote_device_nodemap.FindNode("ExposureTime").SetValue(exposure_time_microseg) # in microseconds  # in microseconds

            return self
        except Exception as e:
            print('No device is free and available. ERR:' + str(e))
            ids_peak.Library.Close()

    def get_image(self):
        """
        Triggers the camera and gets a picture of type Image
        """
        try:
            # trigger image
            self.remote_device_nodemap.FindNode("TriggerSoftware").Execute()
            buffer = self.datastream.WaitForFinishedBuffer(1000)

            # convert to RGB
            raw_image = ids_ipl_extension.BufferToImage(buffer)
            # for Peak version 2.0.1 and lower, use this function instead of the previous line:
            #raw_image = ids_ipl.Image_CreateFromSizeAndBuffer(buffer.PixelFormat(), buffer.BasePtr(), buffer.Size(), buffer.Width(), buffer.Height())
            color_image = raw_image.ConvertTo(ids_ipl.PixelFormatName_RGB8)
            self.datastream.QueueBuffer(buffer)

            # Imagen de nuestra clase importada Image, puedo guardarla, editarla...
            self.image = Image(color_image.get_numpy_3D())
        except Exception as e:
            print('No device is free and available. ERR:' + str(e))
            ids_peak.Library.Close()

    def close_device(self):
        """
        Closes the libraries, seting free the device in use. 
        """
        ids_peak.Library.Close()

    def auto_exposure_get_image(self, gray_pallete = 75):
        """
        Sets automatically exposure, no matters the extern ilumination conditions, 
        given by the light source.
        Computes the average luminance of the frame and compares to a gray_pallete value.
        Args:
            gray_pallete: Value to compare with the average luminance. 
            --Recommended values:--
            > Fiducials = 50
            > Calibration Dots = ... 100
        https://stackoverflow.com/questions/73611185/automatic-shutter-speed-adjustment-feedback-algorithm-based-on-images

        """
        try:
            self.set_exposure(self.exposure_time_seg)
            self.get_image()

            L1 = np.mean(self.image.image) # Compute the average luminance of the current frame 
            print(L1)

            L2 = gray_pallete # Gray Card reference

            a = 0.5  # a = 0.5 parameter is tuneable

            #  Compute exposure Value
            # EV = np.log2(L1)/np.log2(L2)
            self.set_exposure(self.exposure_time_seg*(120 / L1) ** a)  

            while np.abs(L1-L2) > 5:
                self.get_image() 
                L1 = np.mean(self.image.image)
                self.set_exposure(self.exposure_time_seg*(L2 / L1) ** a) 
            self.get_image() 
            self.image.display()
        except Exception as e:
            print('No device is free and available. ERR:' + str(e))
            ids_peak.Library.Close()


