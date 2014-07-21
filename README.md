starboard-lsadrv
================

(Hopefully) Fixed version of the Hitachi StarBoard kernel driver for Linux (lsadrv)

Patched version of lsadrv with 3.10 and 64-bit fixes.

Work sponsored by [Talaria](http://www.talaria.fr/).

Previous works
--------------

* [ALT Linux patches](http://packages.altlinux.org/en/Sisyphus/srpms/kernel-modules-lsadrv-std-pae)
    * lsadrv-build-3.10.patch (fixes build but introduces a crash)
    * ioctl_and_mutex.patch (fixes for ioctl signatures and deprecated init_MUTEX)
