
The original source code of this package is from  Soumyendu Sarkar of LSI, Inc.,
which absorded the former AgereSystems Inc. README and LICENSE are the original 
Agere Systems Inc. documents.  The LSI code version is 2.1.80.

The modem chipsets in principle supported are listed in Supported.txt.  
The crucial newest support is for the 11c11040 modem chip hosted on the subsystems
of many High Definition Audio cards. Support for older modem chipsets has not been 
confirmed for newer Linux kernels.

The agrmodemlib.o_shipped was precompiled at LSI Inc., with a 32 bit compiler.  
Hence the current release is not compatible with x86_64 (64 bit) operating systems, 
though x86_64 motherboards general support 32 bit emualtions under which the modems 
should function.

The 11c11040 support was originally for a Red Star Linux release with 2.6.20 kernel.
Updates to maintain compatibility with emerging kernels have been done by several volunteers.
The most recent patches are from Nikolay Zhuravlev <xxor@mail.ru> supporting
compiling into 2.6.32 kernels..

There is included Proprietary USB modem code. This blocks legal compiling
under Linux, since 2.6.25 kernels.  A USB code free variant has been
requested from LSI maintainer Soumyendu Sarkar.  But pending its arrival the 
Proprietary LICENSE in agrsoftmodem.c has been replaced:
$ grep LICENSE agrsoftmodem.c
//MODULE_LICENSE("Proprietary");
MODULE_LICENSE("GPL");

Older and related code packages are at http://linmodems.technion.ac.il/packages/ltmodem/11c11040/
Thereat the agrsm-tools packages provide for some automation and testing.

The preferred mode of using this package is:
1) First install DKMS support for your Linux distribution
2) Copy this source package to /usr/src/agrsm-11c11040-2.1.80~20091225
3) Complete compiling support for your System.
4) Then the following command will drive the compilation and installation,
with DKMS also providing auto updates to newer kernels.  Ubuntu users must prefix the 
following commands with "sudo".
# ./dodkms
 
Alternatively, DKMS support can be skipped and a more manual installation done,
for each boot kernel as illustrated:

# make clean
make -C /lib/modules/2.6.28-16-generic/build M=/usr/src/agrsm-11c11040-2.1.80~20091225 clean
make[1]: Entering directory `/usr/src/linux-headers-2.6.32-9-generic'
make[1]: Leaving directory `/usr/src/linux-headers-2.6.32-9-generic'

# make 
make -C /usr/src/linux-headers-2.6.32-9-generic M=/usr/src/agrsm-11c11040-2.1.80~20091225 modules
make[1]: Entering directory `/usr/src/linux-headers-2.6.32-9-generic'
  CC [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/agrsoftmodem.o
/usr/src/agrsm-11c11040-2.1.80~20091225/agrsoftmodem.c: In function ‘x_linux_dbg_print_crit’:
/usr/src/agrsm-11c11040-2.1.80~20091225/agrsoftmodem.c:358: warning: the frame size of 1540 bytes is larger than 1024 bytes
/usr/src/agrsm-11c11040-2.1.80~20091225/agrsoftmodem.c: In function ‘x_linux_dbg_print’:
/usr/src/agrsm-11c11040-2.1.80~20091225/agrsoftmodem.c:346: warning: the frame size of 1540 bytes is larger than 1024 bytes
  SHIPPED /usr/src/agrsm-11c11040-2.1.80~20091225/agrmodemlib.o
  CC [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/HDA.o
  CC [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/serial26.o
  LD [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/agrmodem.o
  LD [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/agrserial.o
  Building modules, stage 2.
  MODPOST 2 modules
WARNING: could not find /usr/src/agrsm-11c11040-2.1.80~20091225/.agrmodemlib.o.cmd for /usr/src/agrsm-11c11040-2.1.80~20091225/agrmodemlib.o
  CC      /usr/src/agrsm-11c11040-2.1.80~20091225/agrmodem.mod.o
  LD [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/agrmodem.ko
  CC      /usr/src/agrsm-11c11040-2.1.80~20091225/agrserial.mod.o
  LD [M]  /usr/src/agrsm-11c11040-2.1.80~20091225/agrserial.ko
make[1]: Leaving directory `/usr/src/linux-headers-2.6.32-9-generic'

Skipped the final:
# make modules_install

Marvin.Stodolsky@gmail.com   20091225

