starboard-lsadrv
================

(Hopefully) Fixed version of the Hitachi StarBoard kernel driver for Linux (lsadrv)

Patched version of lsadrv with 3.10 and 64-bit fixes.

Work sponsored by [Talaria](http://www.talaria.fr/).

Installation procedure
----------------------

Either run those steps manually or as a script (not yet fully tested).

```sh
#!/bin/sh -e
# -e stops at first failing command

# Install all the needed 32bit packages for the StarBoard software to run
# libjpeg62 is declared as dependency by the package but not the rest...
sudo apt-get install libjpeg62:i386 libxtst6:i386 libusb-0.1-4:i386 libstdc++6:i386 libfreetype6:i386 libsm6:i386 libglib2.0-0:i386 libxrender1:i386 libfontconfig1:i386 libqtgui4:i386

# Get the official archive and install the .deb package.
wget http://starboard.hitachi-software.co.uk/SoftwareDownload/download/Linux/SBS0942_LINUX.zip
unzip SBS0942_LINUX.zip
cd SBS0942_LINUX/StarBoardSoftware
sudo dpkg -i StarBoardSoftware_9.42_i586.deb
# This should give a warning:
# "None of the pre-compiled kernel modules seems to be compatible with your operating system."

# Finish the installation by running the script manually.
# It will complain about not being able to build the module.
# It might also complain about the coach module, as this one also needs fixing.
sudo /usr/local/StarBoardSoftware/install.sh

# Now compile and install the fixed driver.
sudo apt-get install git
git clone https://github.com/mmuman/starboard-lsadrv.git
cd starboard-lsadrv/lsadrv
make
sudo cp `uname -r`/lsadrv.ko /lib/modules/`uname -r`/kernel/drivers/usb/input/
sudo depmod
sudo modprobe lsadrv

# Start the starboard daemons.
sudo service starboardservice start
```

Previous works
--------------

* [ALT Linux patches](http://packages.altlinux.org/en/Sisyphus/srpms/kernel-modules-lsadrv-std-pae)
    * lsadrv-build-3.10.patch (fixes build but introduces a crash)
    * ioctl_and_mutex.patch (fixes for ioctl signatures and deprecated init_MUTEX)
