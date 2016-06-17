# LIS3DH
*LIS3DH Accelerometer driver for Particle*

This is modeled after my [ADXL362] (https://github.com/rickkas7/ADXL362DMA) driver for the Particle Photon, Electron, etc. except it doesn't have the DMA support written yet. 

This is primarily because the LIS3DH is the accelerometer on the Particle Asset Tracker for the Electron, and presumably most people will use for the wake on movement feature, which is implemented.

There is also a call to get the temperature, which is not particularly accurate but sort of works.

