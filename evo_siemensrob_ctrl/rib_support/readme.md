# RIB environment installation

## System requirements

### Hardware

- IPC427E with CP1625 / CPU1515S PC2

### Software

- Linux OS

To be able to generate and compile the Linux applications the following
software has to be installed in the system, with at least the mentioned versions:

- cmake version 3.13
- gcc version 6.3.0

> **Note:** The **Tips** section at the end of the document describes 
> how to install the software .

To be able to communicate the PLC with Linux Apps:

- CPU1507S F / CPU1505S F
- RT-Preempt patch (optional to perform realtime tasks)

## Installation of RIBInstall

To install the RIB environment the next steps should be followed:

- copy the file *RIBInstall* to the target system.
- open a shell and navigate to where the mentioned file is located
- give execution rights to the file

```bash
chmod +x RIBInstall
```

- execute the file with root rights

```bash
sudo ./RIBInstall
```

## Starting the RIB_App

Once the *ribenvironment* package is installed in SIOS or Debian-based OS, there exist two ways to start the RIB_App.

1. Open a shell and type RIB_App

```bash
pt1@debian:~$ RIB_App
../../Apps/RIB_App/src/main.cpp(119) : main() : LOG: |---------------------------------------|
../../Apps/RIB_App/src/main.cpp(120) : main() : LOG: | This is the RIB APP
../../Apps/RIB_App/src/main.cpp(121) : main() : LOG: | Version DEVELOPER_VERSION
../../Apps/RIB_App/src/main.cpp(122) : main() : LOG: |---------------------------------------|
../../Apps/RIB_App/src/main.cpp(123) : main() : LOG: | RIB_SUPPORT Library Version: DEVELOPER_VERSION
../../Apps/RIB_App/src/main.cpp(124) : main() : LOG: | SegmentLifeTime: 10
../../Apps/RIB_App/src/main.cpp(125) : main() : DBG: | RIB_App PID: 1675
../../Apps/RIB_App/src/main.cpp(126) : main() : LOG: |---------------------------------------|
../../libs/rib_support/src/socketHandler.cpp(59) : setupServerSocket() : DBG: socketFd (constructor): 3
../../Apps/RIB_App/src/main.cpp(136) : main() : LOG: Setup server socket completed successfuly.
../../libs/rib_support/src/socketHandler.cpp(119) : serverWaitForConnection() : DBG: Server-socket listening for connections : 3
```

or

2. Look for the menu entry **RIB App** (see image below) and click it.

![RIB App's icon](doc/img/rib_icon.png)

A terminal will open as you can see below.

![Terminal running the RIB_App](doc/img/terminal.png)

## Build sample Linux applications

> **Attention:**
Make sure that the IPC/OC2 has a correct date set otherwise there will be errors during the 
cmake generation.

To build the sample C++ apps navigate to */home/RIB/template* and execute
the shell script *cmake_build_executables_linux.sh* as follows.

```bash
pt1@debian:/home/RIB/template$ ./cmake_build_executables_linux.sh 
```

After a successful build process the following message is expected:

```bash
Generation finished successfully.
```

The built applications can be found under */home/RIB/template/build/Release/Apps*.

## Test sample applications

### Linux sample applications

To test the Linux only applications follow the next steps:

- start the **RIB_App** as described previously
- start the **C++ provider** application

```bash
pt1@debian:/home/RIB/template/build/Release/apps/Cpp_Provider_App$ ./Provider_App 
```

- start the **C++ consumer** application

```bash
pt1@debian:/home/RIB/template/build/Release/apps/Cpp_Consumer_App$ ./Consumer_App
```

If everything went as expected you should be able to see that the consumer
application prints the values that it reads from the shared memory in the console,
additionally a text file will be created where these values will be logged.
The text file can be found next to the application.

```bash
...
payload: 131
payload: 224
payload: 60
payload: 152
payload: 246
payload: 82
payload: 175
payload: 12
...
```

- start the **Python consumer** application

> **Note:** To be able to start the Python app the following Python packages are needed.
> See the **Tips** sections at the end of this document to see how.
> - cppyy
> - numpy

```bash
pt1@debian:/home/RIB/template/apps/Python_Consumer_App$ python3 consumer.py
```

If everything went as expected you should be able to see that the consumer
application prints the values that it reads from the shared memory in the console,
additionally a text file will be created where these values will be logged.
The text file can be found next to the application.

```bash
...
[14]
[73]
[135]
[193]
[251]
[54]
[111]
[170]
...
```

### PLC - Linux sample applications

To test the Linux applications which share/read  data with/from the PLC follow the next steps:

- load the TIA Portal project to the PLC and addapt it according to your needs

  - check that the Runtime communication interface's IP address of the PLC is in the same subnet as the Linux' VNIC
    > **Note:** See the **Tips** sections at the end of this document to see how to get the Linux VNIC's address
  - check that the IP address of the Linux VNIC is the same as the one under the sock_conf.RemoteAddress variable in the RIB_Connect FB's block interface
  - configure the cyclic time parameter of the cyclic interrupt OB (OB31) according to your needs. This parameter defines how often the shared data will be read/written
  - set an appropiate priority to the cyclic interrupt OB (24 is recommended for real time applications)

- start the **RIB_App** as described previously
- set the **PLC** to run and connect it to the **RIB_App** by setting the ***START_RIB_CONNECTION*** tag to true.
- start the **SWCPU_DataExchange** application

```bash
pt1@swcpu-oc2:/home/RIB/template/build/Release/Apps/SWCPU_DataExchange$ ./SWCPU_DataExchange  
```

> **Note:** For more details check the document HowToUseSWCPUwithRIB.md under *doc/manuals*.

## Tips

- To set the date and time in Debian use the following command:

```bash
pt1@debian:~$ sudo date -s "2020-05-28 09:00"
[sudo] password for pt1:
Thu May 28 09:00:00 CEST 2020
```

- To add a user to the sudoers group use the following command:

```bash
xyz@debian:~$ su
Password:
root@debian:/home/xyz# usermod -aG sudo <username>
```

Restart the OS to complete the operation.

- Install the following packages to make RIB development possible
  
<pre>
sudo apt update
sudo apt install build-essential
sudo apt-get install python python3 python-pip python3-pip
</pre>

- Optionally you can install gdb for debugging

<pre>
sudo apt install gdb
</pre>


- Download and install a cmake version different than the one that is included in your distribution's repository (in this example cmake 3.14.5 will be installed)

<pre>
sudo apt remove --purge --auto-remove cmake
version=3.14
build=5
mkdir ~/temp
cd ~/temp
wget https://cmake.org/files/v$version/cmake-$version.$build.tar.gz
tar -xzvf cmake-$version.$build.tar.gz
cd cmake-$version.$build/
./bootstrap
make -j4
sudo make install
cmake --version
</pre>

- Install Python packages to run the Python consumer application
  
<pre>
pip3 install numpy
pip3 install cppyy
</pre>

- Get the Linux VNIC's address

![Getting VNIC's address](doc/img/ipa.png)

- Delete VMM shared memories
 
To delete VMM shared memories execute the following command as shown below

```bash
pt1@swcpu-oc2:/home$ VMM_SHMEM_Cleaner 
VMM Shared Memory: odkp
Opening vmm shared memory...
Mapping vmm shared memory...
Setting all memory to zero...
Done.
```